/*
 * The qualifier brain: a finite state machine (FSM).
 *
 *   WAIT -> ARM -> DIVE -> SEARCH -> APPROACH -> COMMIT -> DONE
 *                    \________\__________\__________\____-> ABORT
 *
 * Ten times a second step() runs. It looks at which state we are in and calls
 * that state's function. A state function does two things:
 *   1. says what the vehicle should do right now      -> command(...)
 *   2. decides whether it is time to move on          -> transitionTo(...)
 *
 * In:   /mavros/state                     armed? mode? connected?
 *       /mavros/global_position/rel_alt   depth
 *       /controller/debug                 current heading
 *       /qually/gate                      what the detector sees
 *
 * Out:  /controller/target                what we want the vehicle to do
 *       /qually/state                     the current state name (for the
 *                                         monitor and the bag)
 *
 * When it reaches DONE or ABORT it stops the vehicle, disarms and exits. The
 * launch file sees the exit and shuts everything else down.
 */

#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

#include "interfaces/msg/control_target.hpp"
#include "interfaces/msg/controller_debug.hpp"
#include "interfaces/msg/gate_detection.hpp"

namespace qually
{

enum class State
{
  WAIT,       // everything alive yet?
  ARM,        // ALT_HOLD + arm, remember which way is "straight"
  DIVE,       // go down to target depth
  SEARCH,     // drive straight until the gate is seen
  APPROACH,   // drive at the gate, keep it centred
  COMMIT,     // too close to see both poles: lock heading, drive blind
  TURN,       // after the first pass: turn 180 on the spot, then SEARCH again
  DONE,       // stop, disarm, exit 0
  ABORT,      // something went wrong: stop, disarm, exit 1
};

const char * toString(State state);

class QuallyFsm : public rclcpp::Node
{
public:
  QuallyFsm();

  // 0 = finished through DONE, 1 = finished through ABORT.
  int exitCode() const {return exit_code_;}

private:
  // ---- the loop ----------------------------------------------------------
  void step();
  void transitionTo(State next, const std::string & why);

  // ---- one function per state (qually_fsm.cpp) ---------------------------
  void stateWait();       // written
  void stateArm();        // written
  void stateDive();       // TODO(lina)
  void stateSearch();     // TODO(lina)
  void stateApproach();   // TODO(lina)
  void stateCommit();     // TODO(lina)
  void stateTurn();       // TODO(lina)
  void stateFinish();     // written, shared by DONE and ABORT

  // ---- helpers you can use inside a state --------------------------------

  // Seconds since we entered the current state.
  double timeInState() const;

  // Seconds since the run started (since leaving WAIT).
  double timeInRun() const;

  // Tell the controller what to do. Efforts are -1 to +1.
  // vertical: + is up, - is down, 0 = hold depth (in ALT_HOLD).
  void command(double heading_deg, double forward, double vertical = 0.0);

  // Zero everything, no heading hold.
  void commandStop();

  // True if the detector has published within gate_lost_sec.
  bool gateFresh() const;

  // True if the newest detection is fresh AND says found.
  bool gateVisible() const;

  // ---- talking to ArduSub -------------------------------------------------
  void requestMode(const std::string & mode);
  void requestArm(bool arm);

  // ---- what we know about the world (updated by subscribers) -------------
  mavros_msgs::msg::State mavros_state_;      // .connected .armed .mode
  bool have_mavros_state_ = false;

  double depth_m_ = 0.0;                      // positive = below the surface
  bool have_depth_ = false;

  double heading_deg_ = 0.0;                  // compass, 0 to 360
  // Durations and timeouts are measured on the steady clock, which only ever
  // counts forwards. The normal clock (now()) jumps when the computer syncs
  // its time over the network, which would fire or freeze every timeout.
  mutable rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time steadyNow() const {return steady_clock_.now();}

  rclcpp::Time heading_time_;
  bool have_heading_ = false;

  interfaces::msg::GateDetection gate_;       // newest detection
  rclcpp::Time gate_time_;
  bool have_gate_ = false;
  int gate_seen_streak_ = 0;                  // frames in a row with found=true
  int depth_ok_ticks_ = 0;
  bool underwater_ = false;                   // WAIT: start trigger, deep enough right now?
  rclcpp::Time underwater_since_;
  int gate_lost_ticks_ = 0;                   // APPROACH: calls in a row without the gate
  double last_separation_ = 0.0;              // APPROACH: gate width last time we saw it
  int turn_ok_ticks_ = 0;                     // TURN: calls in a row pointing the right way
  int passes_done_ = 0;                       // how many times we went through the gate

  // ---- our own memory ------------------------------------------------------
  State state_ = State::WAIT;
  rclcpp::Time state_entered_;
  rclcpp::Time run_started_;
  rclcpp::Time last_request_;                 // so we don't spam services

  double straight_heading_deg_ = 0.0;         // heading locked in ARM
  double target_heading_deg_ = 0.0;           // heading we are steering to now

  int exit_code_ = 0;

  // ---- numbers from qually.yaml -------------------------------------------
  bool dry_run_;                // true = never arm, never change mode
  bool require_detector_;       // WAIT also waits for /qually/gate
  double start_delay_sec_;      // hands-clear countdown in WAIT
  double start_depth_m_;        // WAIT: start trigger, hold it this deep (0 = off)
  double start_hold_sec_;       // WAIT: ...for this long
  double max_run_sec_;          // whole run longer than this -> ABORT

  double target_depth_m_;       // DIVE: how deep
  double depth_tolerance_m_;    // DIVE: close enough
  double dive_effort_;          // DIVE: how hard to push down (0 to 1)

  double search_effort_;        // SEARCH: forward effort
  int gate_confirm_frames_;     // SEARCH: this many found frames in a row
  double search_timeout_sec_;   // SEARCH: give up after this

  double approach_effort_;      // APPROACH: forward effort
  double steer_gain_deg_;       // APPROACH: degrees of heading per unit offset
  double commit_separation_;    // APPROACH: gate this wide -> COMMIT
  double gate_lost_sec_;        // no detection for this long = lost

  double commit_effort_;        // COMMIT: forward effort
  double commit_sec_;           // COMMIT: how long to drive blind

  // ---- ROS plumbing ---------------------------------------------------------
  rclcpp::Publisher<interfaces::msg::ControlTarget>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rel_alt_sub_;
  rclcpp::Subscription<interfaces::msg::ControllerDebug>::SharedPtr heading_sub_;
  rclcpp::Subscription<interfaces::msg::GateDetection>::SharedPtr gate_sub_;

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace qually
