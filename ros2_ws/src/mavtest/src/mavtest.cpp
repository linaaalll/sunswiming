//!/usr/bin/env

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <ctime>

#include "rclcpp/rclcpp.hpp"

#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include "mavros_msgs/msg/global_position_target.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using ModeResponseFuture = rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFutureWithRequest;
using ArmResponseFuture = rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFutureWithRequest;

// COMPLETELY SYNCHRONOUS NODE :skull_emoji:
class Mavtest : public rclcpp::Node
{
  public:
  Mavtest() : Node("mavtest") {
  //   client_cb_group= this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //  timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    mode_client = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    arm_client = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

    rc_override = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("mavros/rc/override", 10);
    set_depth = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("mavros/setpoint_position/global", 10);
    RCLCPP_INFO(this->get_logger(), "IN TEST NODE");
    sleep(5); // wait for pool guy to set up
    set_z_0 = false;
    setMode(true);
  //  depth_timer = this->create_wall_timer(
  //    100ms, std::bind(&Mavtest::depth_setting_callback, this), timer_cb_group);
  }

  ~Mavtest() {
    set_z_0 = false;
    rc(0,0,0, false);
    setMode(false);
    arm(false);
  }

  // not using as a callback rn
  void depth_setting_callback() {
    if (alt < 0) {
      auto depth_msg = geographic_msgs::msg::GeoPoseStamped();
      // depth_msg.coordinate_frame = 5;
      // depth_msg.type_mask = 4091;

      // using geoposestamped
      // depth_msg.header.stamp = now();
      depth_msg.pose.position.altitude = alt;
      set_depth->publish(depth_msg);
    }
  }

  // mode already set to alt_hold, depth determined
  void run_control() {
    sleep(1); // stay in alt_hold
    RCLCPP_INFO(this->get_logger(), "STARTING CONTROL");

    time_t start, now;
    int count = 0;
    
    time(&start);
    time(&now);
    count = 0;
    while (difftime(now, start) < 2) {
      count++;
      RCLCPP_INFO(this->get_logger(), "%f  seconds passed %d", difftime(now, start), count);
      rc(IGNORE, IGNORE, 0.7);
      delay(500);
      time(&now);
    }
    arm(false); // setting mode to manual and disarms
  } 

  void setMode(bool depth_hold=true) {
	RCLCPP_INFO(this->get_logger(), "in setting mode");
    auto msg = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    msg->base_mode = 0;
    if (depth_hold) {
      msg->custom_mode = "ALT_HOLD";
      mode_client->async_send_request(msg, std::bind(&Mavtest::mode_change_depth, this, _1));
      RCLCPP_INFO(this->get_logger(), "waiting for depth_hold mode to be set");
    } else {
      msg->custom_mode = "MANUAL";
      mode_client->async_send_request(msg, std::bind(&Mavtest::mode_change_manual, this, _1));
      RCLCPP_INFO(this->get_logger(), "waiting for manual mode to be set");

    }
  }

  void arm(bool arming) {
    RCLCPP_INFO(this->get_logger(), "in arm/disarm");
    auto msg = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    msg->value = arming;
    if (arming) {
      arm_client->async_send_request(msg, std::bind(&Mavtest::arm_result, this, _1));
      RCLCPP_INFO(this->get_logger(), "waiting to arm");
    } else {
      arm_client->async_send_request(msg, std::bind(&Mavtest::disarm_result, this, _1));
      RCLCPP_INFO(this->get_logger(), "waiting to disarm");
    }
  }

  void rc(double x, double y, double r, bool led=true) {
    std::vector<int> channels;
    auto msg = mavros_msgs::msg::OverrideRCIn();
    for (int i = 0; i < 18; i++) {
      msg.channels[i] = IGNORE;
    }
    if (x != IGNORE) {
      msg.channels[4] = x*500 + 1500;
    }
    if (y != IGNORE) {
      msg.channels[5] = y*500 + 1500;
    }
    if (r != IGNORE) {
      msg.channels[3] = r*500 + 1500;
    }
    if (led) {
      msg.channels[12] = 1900;
    }
    if (set_z_0) {
      msg.channels[2] = 0;
    }
    RCLCPP_INFO(this->get_logger(), "sending rc commands x: %f, y: %f, r: %f", x, y, r);
    rc_override->publish(msg);
  }

  private:
  double alt = 0;
  bool armed = false;
  bool set_z_0 = false;
  rclcpp::CallbackGroup::SharedPtr client_cb_group;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group;
  const static int IGNORE = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
  rclcpp::TimerBase::SharedPtr depth_timer;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override;
  rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr set_depth;
  
  // delay in milliseconds
  void delay(int duration) {
    usleep(duration*1000);
  }
  void arm_result(ArmResponseFuture msg) {
    auto status = (msg.get().second)->success;
    if (status) {
      armed = !armed;
      RCLCPP_INFO(this->get_logger(), "Arm successful");
      run_control();
    } else {
      RCLCPP_INFO(this->get_logger(), "Arm failed");
    }
  }
  void disarm_result(ArmResponseFuture msg) {
    auto status = (msg.get().second)->success;
    if (status) {
      armed = !armed;
      RCLCPP_INFO(this->get_logger(), "Disarm successful");
    } else {
      RCLCPP_INFO(this->get_logger(), "Disarm failed");
    }
  }
  void mode_change_depth(ModeResponseFuture msg) {
    auto status = (msg.get().second)->mode_sent;
    if (status) {
      RCLCPP_INFO(this->get_logger(), "Mode successfully changed to alt_hold!");
      alt = -0.5;
      depth_setting_callback();
      arm(true);
    } else {
      RCLCPP_INFO(this->get_logger(), "Mode could not be changed to alt_hold");
    }
  }
  void mode_change_manual(ModeResponseFuture msg) {
    auto status = (msg.get().second)->mode_sent;
    if (status) {
      RCLCPP_INFO(this->get_logger(), "Mode successfully changed to manual!");
      arm(false);
    } else {
      RCLCPP_INFO(this->get_logger(), "Mode could not be changed to manual");
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(std::make_shared<MavTest>());
  // executor.spin();
  rclcpp::spin(std::make_shared<Mavtest>());
  rclcpp::shutdown();
  return 0;
}
