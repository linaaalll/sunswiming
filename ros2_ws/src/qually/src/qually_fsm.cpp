#include "qually/qually_fsm.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace qually
{

const char * toString(State state)
{
  switch (state) {
    case State::WAIT: return "WAIT";
    case State::ARM: return "ARM";
    case State::DIVE: return "DIVE";
    case State::SEARCH: return "SEARCH";
    case State::APPROACH: return "APPROACH";
    case State::COMMIT: return "COMMIT";
    case State::TURN: return "TURN";
    case State::DONE: return "DONE";
    case State::ABORT: return "ABORT";
  }
  return "?";
}

QuallyFsm::QuallyFsm()
: Node("qually_fsm")
{
  // Every number here can be changed in launch_file/config/qually.yaml.
  dry_run_ = declare_parameter<bool>("dry_run", true);
  require_detector_ = declare_parameter<bool>("require_detector", true);
  start_delay_sec_ = declare_parameter<double>("start_delay_sec", 5.0);
  sensor_rate_hz_ = declare_parameter<double>("sensor_rate_hz", 20.0);
  max_run_sec_ = declare_parameter<double>("max_run_sec", 120.0);

  target_depth_m_ = declare_parameter<double>("target_depth_m", 0.8);
  depth_tolerance_m_ = declare_parameter<double>("depth_tolerance_m", 0.1);
  dive_effort_ = declare_parameter<double>("dive_effort", 0.3);
  depth_mode_ = declare_parameter<std::string>("depth_mode", "ALT_HOLD");
  depth_kp_ = declare_parameter<double>("depth_kp", 1.0);
  depth_ki_ = declare_parameter<double>("depth_ki", 0.3);
  depth_max_effort_ = declare_parameter<double>("depth_max_effort", 0.6);

  search_effort_ = declare_parameter<double>("search_effort", 0.2);
  gate_confirm_frames_ = declare_parameter<int>("gate_confirm_frames", 5);
  search_timeout_sec_ = declare_parameter<double>("search_timeout_sec", 40.0);

  approach_effort_ = declare_parameter<double>("approach_effort", 0.25);
  steer_gain_deg_ = declare_parameter<double>("steer_gain_deg", 15.0);
  commit_separation_ = declare_parameter<double>("commit_separation", 0.7);
  gate_lost_sec_ = declare_parameter<double>("gate_lost_sec", 1.0);

  commit_effort_ = declare_parameter<double>("commit_effort", 0.3);
  commit_sec_ = declare_parameter<double>("commit_sec", 6.0);

  target_pub_ = create_publisher<interfaces::msg::ControlTarget>("/controller/target", 10);
  state_pub_ = create_publisher<std_msgs::msg::String>("/qually/state", 10);

  mavros_state_sub_ = create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", 10,
    [this](mavros_msgs::msg::State::SharedPtr msg) {
      mavros_state_ = *msg;
      have_mavros_state_ = true;
    });

  // rel_alt is metres above where ArduSub booted, so underwater it is
  // negative. Flip it so depth is a positive number.
  rel_alt_sub_ = create_subscription<std_msgs::msg::Float64>(
    "/mavros/global_position/rel_alt", rclcpp::SensorDataQoS(),
    [this](std_msgs::msg::Float64::SharedPtr msg) {
      depth_m_ = -msg->data;
      have_depth_ = true;
    });

  // The controller already works out the compass heading, reuse it.
  heading_sub_ = create_subscription<interfaces::msg::ControllerDebug>(
    "/controller/debug", 10,
    [this](interfaces::msg::ControllerDebug::SharedPtr msg) {
      heading_deg_ = msg->measurement;
      heading_time_ = steadyNow();
      have_heading_ = true;
    });

  gate_sub_ = create_subscription<interfaces::msg::GateDetection>(
    "/qually/gate", 10,
    [this](interfaces::msg::GateDetection::SharedPtr msg) {
      gate_ = *msg;
      gate_time_ = steadyNow();
      have_gate_ = true;
      gate_seen_streak_ = msg->found ? gate_seen_streak_ + 1 : 0;
    });

  mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  rate_client_ = create_client<mavros_msgs::srv::MessageInterval>("/mavros/set_message_interval");

  state_entered_ = steadyNow();
  run_started_ = steadyNow();
  last_request_ = steadyNow();
  heading_time_ = steadyNow();
  gate_time_ = steadyNow();

  timer_ = create_wall_timer(100ms, [this]() {step();});

  RCLCPP_INFO(
    get_logger(), "qually fsm up. dry_run=%s%s",
    dry_run_ ? "true" : "false",
    dry_run_ ? " (will NOT arm or change mode)" : " (WILL ARM, keep clear)");
}

// ---------------------------------------------------------------------------
// the loop
// ---------------------------------------------------------------------------

void QuallyFsm::step()
{
  // Safety checks that apply while the run is live, whatever the state.
  const bool live =
    state_ == State::DIVE || state_ == State::SEARCH ||
    state_ == State::APPROACH || state_ == State::COMMIT ||
    state_ == State::TURN;

  if (live) {
    if (timeInRun() > max_run_sec_) {
      transitionTo(State::ABORT, "run took longer than max_run_sec");
    } else if (!dry_run_ && !mavros_state_.connected) {
      transitionTo(State::ABORT, "lost the flight controller");
    } else if (!dry_run_ && !mavros_state_.armed) {
      transitionTo(State::ABORT, "vehicle got disarmed by something else");
    }
  }

  switch (state_) {
    case State::WAIT: stateWait(); break;
    case State::ARM: stateArm(); break;
    case State::DIVE: stateDive(); break;
    case State::SEARCH: stateSearch(); break;
    case State::APPROACH: stateApproach(); break;
    case State::COMMIT: stateCommit(); break;
    case State::TURN: stateTurn(); break;
    case State::DONE: stateFinish(); break;
    case State::ABORT: stateFinish(); break;
  }

  std_msgs::msg::String msg;
  msg.data = toString(state_);
  state_pub_->publish(msg);
}

void QuallyFsm::transitionTo(State next, const std::string & why)
{
  RCLCPP_INFO(
    get_logger(), "STATE %s -> %s after %.1fs: %s",
    toString(state_), toString(next), timeInState(), why.c_str());
  state_ = next;
  state_entered_ = steadyNow();
  last_request_ = steadyNow() - rclcpp::Duration(10s);   // allow a request straight away
}

// ---------------------------------------------------------------------------
// states that are written for you (the safety-critical plumbing)
// ---------------------------------------------------------------------------

void QuallyFsm::stateWait()
{
  commandStop();

  // As soon as the flight controller is there, ask it for faster data (once).
  if (!rates_requested_ && have_mavros_state_ && mavros_state_.connected) {
    requestSensorRates();
  }

  const char * missing = nullptr;
  if (!have_mavros_state_ || !mavros_state_.connected) {
    missing = "mavros not connected to the flight controller";
  } else if (!have_heading_ || (steadyNow() - heading_time_).seconds() > 1.0) {
    missing = "no heading (is heading_controller running? is /mavros/imu/data alive?)";
  } else if (!have_depth_) {
    missing = "no depth on /mavros/global_position/rel_alt";
  } else if (require_detector_ && !gateFresh()) {
    missing = "detector not publishing /qually/gate";
  }

  if (missing) {
    RCLCPP_INFO_THROTTLE(get_logger(), steady_clock_, 2000, "WAIT: %s", missing);
    state_entered_ = steadyNow();              // countdown only runs when all is good
    return;
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(), steady_clock_, 1000,
    "WAIT: all good, starting in %.0fs", start_delay_sec_ - timeInState());

  if (timeInState() >= start_delay_sec_) {
    run_started_ = steadyNow();
    transitionTo(State::ARM, "everything alive");
  }
}

void QuallyFsm::stateArm()
{
  commandStop();

  if (dry_run_) {
    straight_heading_deg_ = heading_deg_;
    target_heading_deg_ = heading_deg_;
    transitionTo(State::DIVE, "dry run, skipped arming");
    return;
  }

  if (timeInState() > 15.0) {
    transitionTo(State::ABORT, "could not get the mode + armed in 15s");
    return;
  }

  // One request per second, mode first, then arm. /mavros/state tells us
  // when each one has really happened.
  const bool can_ask = (steadyNow() - last_request_).seconds() > 1.0;

  if (mavros_state_.mode != depth_mode_) {
    if (can_ask) {requestMode(depth_mode_);}
    return;
  }
  if (!mavros_state_.armed) {
    if (can_ask) {requestArm(true);}
    return;
  }

  // "Straight" is whichever way the vehicle is pointing at this moment, so
  // aim it at the gate before you launch.
  straight_heading_deg_ = heading_deg_;
  target_heading_deg_ = heading_deg_;
  depth_integral_ = 0.0;
  transitionTo(State::DIVE, "armed in " + depth_mode_);
}

void QuallyFsm::stateFinish()
{
  commandStop();
  exit_code_ = (state_ == State::ABORT) ? 1 : 0;

  const bool need_disarm = !dry_run_ && mavros_state_.armed;

  if (need_disarm && timeInState() < 5.0) {
    if ((steadyNow() - last_request_).seconds() > 1.0) {requestArm(false);}
    return;
  }
  if (need_disarm) {
    RCLCPP_ERROR(get_logger(), "STILL ARMED after 5s of asking. Disarm it by hand!");
  }

  // Give the stop command a moment to reach the thrusters, then leave.
  if (timeInState() > 1.0) {
    RCLCPP_INFO(get_logger(), "finished in %s, exiting", toString(state_));
    timer_->cancel();
    rclcpp::shutdown();
  }
}

// ---------------------------------------------------------------------------
// states that are yours
//
// Each one currently just holds still. Nothing bad happens if you leave one
// empty: the vehicle sits there until max_run_sec, then ABORT.
//
// Things you can read:   depth_m_, heading_deg_, gate_ (.found .centre_offset
//                        .separation .confidence), gate_seen_streak_,
//                        straight_heading_deg_, target_heading_deg_,
//                        timeInState(), gateFresh(), gateVisible()
// Things you can do:     command(heading, forward, vertical), commandStop(),
//                        transitionTo(State::X, "why")
// Numbers from yaml:     see the list in qually_fsm.hpp
// ---------------------------------------------------------------------------

void QuallyFsm::stateDive()
{
  // dry run doesn't affect depth
  if (dry_run_) {
    command(straight_heading_deg_, 0.0, 0.0);

    if (timeInState() > 3.0) {
      transitionTo(State::SEARCH, "dry run, skip dive");
    }
    return;
  }

  // find the error
  const double error = target_depth_m_ - depth_m_;

  // how will the rov be pushed
  double vertical = 0.0;
  bool at_depth = std::abs(error) < depth_tolerance_m_;
  if (depth_mode_ == "ALT_HOLD") {
    // ArduSub holds depth, we only nudge the stick up or down
    if (error > depth_tolerance_m_) {
      vertical = -dive_effort_;
    } else if (error < -depth_tolerance_m_) {
      vertical = dive_effort_;
    }
  }
  // any other mode: leave vertical at 0, command() fills in our own depth hold

  // see how long we stay at the depth
  if (at_depth) {
    depth_ok_ticks_ ++;
  } else {
    depth_ok_ticks_ = 0;
  }

  // move on or give up after 20 times
  if (depth_ok_ticks_ >= 20) {
    depth_ok_ticks_ = 0;
    transitionTo(State::SEARCH, "depth steady");
  } else if (timeInState() > 30.0) {
    transitionTo(State::ABORT, "depth failed to reach");
  }

  // let's goooooo
  command(straight_heading_deg_, 0.0, vertical);
}

void QuallyFsm::stateSearch()
{
  command(straight_heading_deg_, search_effort_, 0.0);

  if (gate_seen_streak_ >= gate_confirm_frames_) {
    target_heading_deg_ = heading_deg_;
    gate_lost_ticks_ = 0;
    last_separation_ = 0.0;
    transitionTo(State::APPROACH, "gate confirmed");
  } else if (timeInState() > search_timeout_sec_) {
    transitionTo(State::ABORT, "no gate found");
  }
}

void QuallyFsm::stateApproach()
{
  // let's look for the gate!
  if (gateVisible()) {
    gate_lost_ticks_ = 0;
    last_separation_ = gate_.separation;
    // aiming for the gate here
    target_heading_deg_ = std::fmod(
      heading_deg_ + steer_gain_deg_ * gate_.centre_offset + 360.0, 360.0);
  } else {
    // oh no where's my gate
    gate_lost_ticks_++;
  }

  command(target_heading_deg_, approach_effort_, 0.0);

  if (gateVisible() && gate_.separation >= commit_separation_) {
    transitionTo(State::COMMIT, "gate is in view");
  } else if (gate_lost_ticks_ > gate_lost_sec_ * 10) {
    if (last_separation_ > 0.6 * commit_separation_) {
      transitionTo(State::COMMIT, "cannot see the gate, but we are close");
    } else {
      transitionTo(State::SEARCH, "gate lost, gg gang");
    }
  }

  command(target_heading_deg_, 0.0, 0.0);
}

void QuallyFsm::stateCommit()
{
  command(target_heading_deg_, commit_effort_, 0.0);

  if (timeInState() > commit_sec_) {
    passes_done_ ++;
    if (passes_done_ >= 2) {
      transitionTo(State::DONE, "through the gate twice");
    } else {
      transitionTo(State::TURN, "first pass complete");
    }
  }
}

void QuallyFsm::stateTurn()
{
  // figure out hoe much I want to turn.
  const double turn_heading = std::fmod(straight_heading_deg_ + 180.0,
    360.0);
  command(turn_heading, 0.0, 0.0);

  // let's give a small calibration for it to turn straight
  const double error = std::remainder(turn_heading - heading_deg_,
    360.0);

  if (std::abs(error) < 5.0) {
    turn_ok_ticks_ ++;
  } else {
    turn_ok_ticks_ = 0;
  }

  if (turn_ok_ticks_ >= 10) {
    turn_ok_ticks_ = 0;
    straight_heading_deg_ = turn_heading;
    transitionTo(State::SEARCH, "turned around");
  } else if (timeInState() > 20.0) {
    transitionTo(State::ABORT, "turn never settled");
  }
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

double QuallyFsm::timeInState() const
{
  return (steadyNow() - state_entered_).seconds();
}

double QuallyFsm::timeInRun() const
{
  return (steadyNow() - run_started_).seconds();
}

void QuallyFsm::command(double heading_deg, double forward, double vertical)
{
  // Nobody asked for a vertical push: hold depth. In ALT_HOLD that means
  // sending 0 (ArduSub does it). In any other mode we do it ourselves.
  if (vertical == 0.0) {
    vertical = holdDepth();
  }

  interfaces::msg::ControlTarget msg;
  msg.hold_heading = true;
  msg.heading_deg = heading_deg;
  msg.forward = forward;
  msg.vertical = vertical;
  target_pub_->publish(msg);
}

// Our own depth hold, for when ArduSub's ALT_HOLD is too gentle to pull the
// vehicle under (pool run, 13 Nov: it only ever used about a third of the
// down-thrust and the vehicle stayed at the surface).
// Called once per tick (10 Hz). Returns the vertical effort, negative = down.
double QuallyFsm::holdDepth()
{
  if (depth_mode_ == "ALT_HOLD" || dry_run_) {
    return 0.0;
  }
  const double error = target_depth_m_ - depth_m_;          // + = need to go deeper

  // I: slowly learns how much push it takes just to stay down (buoyancy).
  // Capped so it can never ask for more than depth_max_effort on its own.
  const double integral_limit = depth_ki_ > 0.0 ? depth_max_effort_ / depth_ki_ : 0.0;
  depth_integral_ = std::clamp(depth_integral_ + error * 0.1, -integral_limit, integral_limit);

  const double push = depth_kp_ * error + depth_ki_ * depth_integral_;   // + = push down
  return -std::clamp(push, -depth_max_effort_, depth_max_effort_);
}

void QuallyFsm::commandStop()
{
  target_pub_->publish(interfaces::msg::ControlTarget());   // all zero / false
}

bool QuallyFsm::gateFresh() const
{
  return have_gate_ && (steadyNow() - gate_time_).seconds() < gate_lost_sec_;
}

bool QuallyFsm::gateVisible() const
{
  return gateFresh() && gate_.found;
}

// By default ArduSub sends depth about 3 times a second and attitude about 9.
// That is slow for holding depth and heading ourselves, so ask for more.
// Depth comes from GLOBAL_POSITION_INT (id 33): ArduSub fills it from the
// external pressure sensor. Attitude is ATTITUDE (id 30).
// This only changes how often it talks, nothing about how it flies, and it is
// forgotten when the flight controller restarts.
void QuallyFsm::requestSensorRates()
{
  rates_requested_ = true;
  if (sensor_rate_hz_ <= 0.0) {
    return;
  }
  if (!rate_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "/mavros/set_message_interval not available, sensor rates left as they are");
    return;
  }
  for (const uint32_t id : {33u, 30u}) {
    auto request = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
    request->message_id = id;
    request->message_rate = static_cast<float>(sensor_rate_hz_);
    rate_client_->async_send_request(
      request,
      [this, id](rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedFuture future) {
        RCLCPP_INFO(
          get_logger(), "asked ArduSub for message %u at %.0f Hz: %s", id, sensor_rate_hz_,
          future.get()->success ? "ok" : "REFUSED");
      });
  }
}

void QuallyFsm::requestMode(const std::string & mode)
{
  last_request_ = steadyNow();
  if (!mode_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "/mavros/set_mode not available yet");
    return;
  }
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = mode;
  RCLCPP_INFO(get_logger(), "asking for mode %s", mode.c_str());
  mode_client_->async_send_request(
    request,
    [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      if (!future.get()->mode_sent) {
        RCLCPP_WARN(get_logger(), "ArduSub refused mode %s", mode.c_str());
      }
    });
}

void QuallyFsm::requestArm(bool arm)
{
  last_request_ = steadyNow();
  if (!arm_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "/mavros/cmd/arming not available yet");
    return;
  }
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = arm;
  RCLCPP_INFO(get_logger(), "asking to %s", arm ? "ARM" : "disarm");
  arm_client_->async_send_request(
    request,
    [this, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
      if (!future.get()->success) {
        RCLCPP_WARN(
          get_logger(), "ArduSub refused to %s (check /mavros/statustext/recv for why)",
          arm ? "arm" : "disarm");
      }
    });
}

}  // namespace qually

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<qually::QuallyFsm>();
  rclcpp::spin(node);
  const int code = node->exitCode();
  rclcpp::shutdown();
  return code;
}
