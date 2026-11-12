/*
 * Heading controller: keeps the vehicle pointing where the brain says.
 *
 * In:   /controller/target   interfaces/ControlTarget   what the brain wants
 *       /mavros/imu/data     sensor_msgs/Imu            where we are pointing
 *
 * Out:  cmd_vel              geometry_msgs/Twist        to rc_control
 *       /controller/debug    interfaces/ControllerDebug  for plots and the monitor
 *
 * Forward / lateral / vertical are passed straight through. Only yaw goes
 * through the PID (controller/pid.hpp, which is yours).
 *
 * Sign convention, same as rc_control: angular.z positive = turn RIGHT
 * (compass heading goes up). That is the opposite of normal ROS, so check it
 * on the bench before trusting it.
 *
 * Gains can be changed while it runs, no rebuild, no restart:
 *     ros2 param set /heading_controller kp 0.02
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "interfaces/msg/control_target.hpp"
#include "interfaces/msg/controller_debug.hpp"

#include "controller/pid.hpp"

class HeadingController : public rclcpp::Node
{
public:
  HeadingController()
  : Node("heading_controller")
  {
    declare_parameter<double>("kp", 0.0);
    declare_parameter<double>("ki", 0.0);
    declare_parameter<double>("kd", 0.0);
    declare_parameter<double>("output_limit", 0.5);
    declare_parameter<double>("integral_limit", 0.2);

    rate_hz_ = declare_parameter<double>("rate_hz", 20.0);
    // No target from the brain for this long -> command zero.
    target_timeout_sec_ = declare_parameter<double>("target_timeout_sec", 0.5);
    // No heading from the flight controller for this long -> no yaw command.
    heading_timeout_sec_ = declare_parameter<double>("heading_timeout_sec", 0.5);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    debug_pub_ = create_publisher<interfaces::msg::ControllerDebug>("/controller/debug", 10);

    target_sub_ = create_subscription<interfaces::msg::ControlTarget>(
      "/controller/target", 10,
      [this](interfaces::msg::ControlTarget::SharedPtr msg) {
        target_ = *msg;
        target_time_ = steady_clock_.now();
        have_target_ = true;
      });

    // mavros publishes sensor data "best effort", the subscriber has to match.
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/mavros/imu/data", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        heading_deg_ = compassHeadingDeg(msg->orientation);
        heading_time_ = steady_clock_.now();
        have_heading_ = true;
      });

    last_tick_ = steady_clock_.now();
    target_time_ = last_tick_;
    heading_time_ = last_tick_;

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {tick();});

    RCLCPP_INFO(get_logger(), "heading controller running at %.0f Hz", rate_hz_);
  }

private:
  // mavros gives orientation as a quaternion in ROS axes (yaw 0 = east,
  // counter-clockwise positive). Turn that into a compass heading
  // (0 = north, clockwise positive, 0 to 360).
  static double compassHeadingDeg(const geometry_msgs::msg::Quaternion & q)
  {
    const double yaw_rad = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    double heading = 90.0 - yaw_rad * 180.0 / M_PI;
    heading = std::fmod(heading, 360.0);
    if (heading < 0.0) {
      heading += 360.0;
    }
    return heading;
  }

  void tick()
  {
    const auto stamp = now();                  // wall time, only for message headers
    const auto t = steady_clock_.now();        // steady time, for dt and timeouts
    const double dt = std::max(1e-3, (t - last_tick_).seconds());
    last_tick_ = t;

    // Read the gains every tick so `ros2 param set` takes effect immediately.
    controller::PidGains gains;
    gains.kp = get_parameter("kp").as_double();
    gains.ki = get_parameter("ki").as_double();
    gains.kd = get_parameter("kd").as_double();
    gains.output_limit = get_parameter("output_limit").as_double();
    gains.integral_limit = get_parameter("integral_limit").as_double();
    pid_.setGains(gains);

    const bool target_ok =
      have_target_ && (t - target_time_).seconds() < target_timeout_sec_;
    const bool heading_ok =
      have_heading_ && (t - heading_time_).seconds() < heading_timeout_sec_;

    geometry_msgs::msg::Twist cmd;       // all zeros
    interfaces::msg::ControllerDebug debug;
    debug.header.stamp = stamp;
    debug.measurement = heading_deg_;

    if (target_ok) {
      cmd.linear.x = target_.forward;
      cmd.linear.y = target_.lateral;
      cmd.linear.z = target_.vertical;
    }

    const bool yaw_active = target_ok && heading_ok && target_.hold_heading;

    if (yaw_active) {
      if (!was_active_) {
        pid_.reset();                    // fresh start, no stale integral
      }
      const double error = controller::headingError(target_.heading_deg, heading_deg_);
      const double output = pid_.update(error, dt);

      cmd.angular.z = output;

      debug.enabled = true;
      debug.target = target_.heading_deg;
      debug.error = error;
      debug.p_term = pid_.pTerm();
      debug.i_term = pid_.iTerm();
      debug.d_term = pid_.dTerm();
      debug.output = output;
    }
    was_active_ = yaw_active;

    if (target_ok && target_.hold_heading && !heading_ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 2000,
        "brain wants heading hold but no /mavros/imu/data, yaw is off");
    }

    cmd_pub_->publish(cmd);
    debug_pub_->publish(debug);
  }

  controller::Pid pid_;

  double rate_hz_;
  double target_timeout_sec_;
  double heading_timeout_sec_;

  interfaces::msg::ControlTarget target_;
  // Durations and timeouts are measured on the steady clock, which only ever
  // counts forwards. The normal clock (now()) jumps when the computer syncs
  // its time over the network, which would fire or freeze every timeout.
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time target_time_;
  bool have_target_ = false;

  double heading_deg_ = 0.0;
  rclcpp::Time heading_time_;
  bool have_heading_ = false;

  rclcpp::Time last_tick_;
  bool was_active_ = false;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<interfaces::msg::ControllerDebug>::SharedPtr debug_pub_;
  rclcpp::Subscription<interfaces::msg::ControlTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadingController>());
  rclcpp::shutdown();
  return 0;
}
