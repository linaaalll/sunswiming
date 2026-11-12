/* 
* This node should be the ONLY node that will puublish to mavros/rc/override
* The node will be translating twist into the desired DOF
* 
* Input: geometry_msgs/Twist --> <cmd_vel_topic>
*       linear.x = forward
*       linear.y = lateral
*       linear.z = vertical (+ up). In ALT_HOLD, 0 = hold depth
*       angular.z = yaw
*
* Output: mavros_msgs/OverrideRCIn on mavros/rc/override
*
*
*/

#include <chrono>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"


using namespace std::chrono_literals;


class Controller : public rclcpp::Node
{
public:
  Controller() : Node("nav2_to_mavros_controller")
  {


    // ropic listened to from nav2
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");

    // how often RC ovverides - x times per second 
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 20.0);

    // Max velocities Nav2 might output (used for normalization into [-1, 1])
    max_fwd_vel_ = this->declare_parameter<double>("max_fwd_vel", 0.5);      // m/s
    max_lat_vel_ = this->declare_parameter<double>("max_lat_vel", 0.3);      // m/s
    max_yaw_rate_ = this->declare_parameter<double>("max_yaw_rate", 0.6);    // rad/s
    max_vert_vel_ = this->declare_parameter<double>("max_vert_vel", 0.3);    // m/s

    // Safety: if cmd_vel not received for this long, output neutral RC
    cmd_timeout_sec_ = this->declare_parameter<double>("cmd_timeout_sec", 0.5);


    rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("mavros/rc/override", 10);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&Controller::cmdVelCb, this, std::placeholders::_1));

    last_cmd_time_ = steady_clock_.now();
    last_cmd_ = geometry_msgs::msg::Twist();

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Controller::publishRc, this));

    RCLCPP_INFO(this->get_logger(),
      "Nav2→MAVROS controller running. Subscribing to '%s', publishing to 'mavros/rc/override'",
      cmd_vel_topic_.c_str());
    RCLCPP_INFO(get_logger(),
      "CONTROLLER_CONFIG rate_hz=%.1f timeout_s=%.2f limits=(%.3f,%.3f,%.3f) "
      "PWM=1500+500*effort; zero-based RC indices: forward=4 lateral=5 yaw=3",
      publish_rate_hz_, cmd_timeout_sec_, max_fwd_vel_, max_lat_vel_, max_yaw_rate_);
  }

private:
  static constexpr int IGNORE = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;



// PWM conversion helper 
  static int pwmFromNormalized(double u_norm) {

    u_norm = std::clamp(u_norm, -1.0, 1.0);
    const int pwm = static_cast<int>(1500.0 + u_norm * 500.0);
    
    return std::clamp(pwm, 1000, 2000);
     
  }

// safecheck to prevent rubbish from enteirng rc pwm conversion 
  static double safeDiv(double num, double denom, double fallback = 0.0) {

    if (std::abs(denom) < 1e-9) return fallback;

    return num / denom;
  }

// Store most recent command and timestamp 
  void cmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg) {

    last_cmd_ = *msg;
    last_cmd_time_ = steady_clock_.now();
    if (!have_command_) {
      RCLCPP_INFO(get_logger(), "COMMAND_STREAM first command received on %s", cmd_vel_topic_.c_str());
    }
    have_command_ = true;

  }


// build and publish the rc override 
  void publishRc() {

    const auto now = steady_clock_.now();

    // Timeout check -- is nav2 still commanding? 
    const double age = (now - last_cmd_time_).seconds();
    const bool timed_out = age > cmd_timeout_sec_;
    if (timed_out != was_timed_out_) {
      if (timed_out) {
        RCLCPP_WARN(get_logger(),
          "COMMAND_STREAM timeout age_s=%.3f limit_s=%.3f; publishing neutral PWM",
          age, cmd_timeout_sec_);
      } else {
        RCLCPP_INFO(get_logger(), "COMMAND_STREAM recovered age_s=%.3f", age);
      }
      was_timed_out_ = timed_out;
    }

    double x_norm = 0.0;
    double y_norm = 0.0;
    double r_norm = 0.0;
    double z_norm = 0.0;

    if (!timed_out) {
      // Normalize Twist to [-1, 1] in each axis
      x_norm = safeDiv(last_cmd_.linear.x,  max_fwd_vel_);
      y_norm = safeDiv(last_cmd_.linear.y,  max_lat_vel_);
      r_norm = safeDiv(last_cmd_.angular.z, max_yaw_rate_);
      x_norm = std::clamp(x_norm, -1.0, 1.0);
      y_norm = std::clamp(y_norm, -1.0, 1.0);
      r_norm = std::clamp(r_norm, -1.0, 1.0);
      z_norm = std::clamp(safeDiv(last_cmd_.linear.z, max_vert_vel_), -1.0, 1.0);
    } else {
      // Fail-safe: stop
      x_norm = 0.0; y_norm = 0.0; r_norm = 0.0; z_norm = 0.0;
    }

    mavros_msgs::msg::OverrideRCIn out;
    for (int i = 0; i < 18; i++) out.channels[i] = IGNORE;

    // Match your existing convention:
    // x -> ch4 (forward), y -> ch5 (lateral), r -> ch3 (yaw)
    out.channels[4] = pwmFromNormalized(x_norm);
    out.channels[5] = pwmFromNormalized(y_norm);
    out.channels[3] = pwmFromNormalized(r_norm);
    // z -> ch2 (throttle). 1500 = no vertical thrust in MANUAL,
    // "hold this depth" in ALT_HOLD.
    out.channels[2] = pwmFromNormalized(z_norm);

    rc_pub_->publish(out);
    RCLCPP_INFO_THROTTLE(get_logger(), steady_clock_, 1000,
      "RC_OUTPUT have_command=%s timed_out=%s command_age_s=%.3f "
      "input_forward=%+.3f input_lateral=%+.3f input_yaw=%+.3f "
      "effort_forward=%+.3f effort_lateral=%+.3f effort_yaw=%+.3f "
      "pwm[4]=%u pwm[5]=%u pwm[3]=%u rc_subscribers=%zu",
      have_command_ ? "true" : "false", timed_out ? "true" : "false", age,
      last_cmd_.linear.x, last_cmd_.linear.y, last_cmd_.angular.z,
      x_norm, y_norm, r_norm,
      static_cast<unsigned>(out.channels[4]), static_cast<unsigned>(out.channels[5]),
      static_cast<unsigned>(out.channels[3]), rc_pub_->get_subscription_count());
  }  

  // params that the node keeps for reference 
  std::string cmd_vel_topic_;
  double publish_rate_hz_;
  double max_fwd_vel_;
  double max_lat_vel_;
  double max_yaw_rate_;
  double max_vert_vel_;
  double cmd_timeout_sec_;

  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Durations and timeouts are measured on the steady clock, which only ever
  // counts forwards. The normal clock (now()) jumps when the computer syncs
  // its time over the network, which would fire or freeze every timeout.
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_cmd_time_;
  geometry_msgs::msg::Twist last_cmd_;
  bool have_command_ = false, was_timed_out_ = false;
};

int main(int argc, char ** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();

  return 0;

}