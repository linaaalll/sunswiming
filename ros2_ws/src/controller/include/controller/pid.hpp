/*
 * A plain PID controller. No ROS in here on purpose: it is just maths, so it
 * can be tested on its own and reused for any axis later (heading now, maybe
 * lateral or depth one day).
 *
 * The node that uses it is heading_controller.cpp. Every tick it does:
 *
 *     double error  = headingError(target, measurement);
 *     double output = pid.update(error, dt);
 *
 * The declarations below are the contract. The bodies are in pid.cpp and the
 * interesting ones are left for you.
 */

#pragma once

namespace controller
{

struct PidGains
{
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;

  // The output is clamped to [-output_limit, +output_limit].
  double output_limit = 1.0;

  // The I term on its own is clamped to [-integral_limit, +integral_limit].
  // This is the anti-windup: without it the integral keeps growing while the
  // vehicle physically can't keep up, then overshoots for ages.
  double integral_limit = 0.3;
};

class Pid
{
public:
  void setGains(const PidGains & gains) {gains_ = gains;}

  // Forget everything (integral, previous error). Called whenever the brain
  // switches heading hold on, so old history doesn't kick the vehicle.
  void reset();

  // One step of the controller.
  //   error: target - measurement, already wrapped (degrees for heading)
  //   dt:    seconds since the previous call (always > 0)
  // Returns the output, clamped to +/- output_limit.
  // Must also store p_term_, i_term_, d_term_ so they can be published.
  double update(double error, double dt);

  double pTerm() const {return p_term_;}
  double iTerm() const {return i_term_;}
  double dTerm() const {return d_term_;}

private:
  PidGains gains_;

  double integral_ = 0.0;      // running sum of error * dt
  double prev_error_ = 0.0;    // error on the previous call
  bool has_prev_ = false;      // false on the first call after reset()

  double p_term_ = 0.0;
  double i_term_ = 0.0;
  double d_term_ = 0.0;
};

// The shortest way round from current to target, in degrees.
// Always in [-180, +180]. Positive = need to turn right (heading goes up).
//
//   headingError( 90,  80) ->  +10
//   headingError( 10, 350) ->  +20   (not -340)
//   headingError(350,  10) ->  -20   (not +340)
double headingError(double target_deg, double current_deg);

}  // namespace controller
