#include "controller/pid.hpp"

#include <algorithm>
#include <cmath>

namespace controller
{

void Pid::reset()
{
  integral_ = 0.0;
  prev_error_ = 0.0;
  has_prev_ = false;
  p_term_ = 0.0;
  i_term_ = 0.0;
  d_term_ = 0.0;
}

double Pid::update(double error, double dt)
{
  // P term
  p_term_ = gains_.kp * error;

  // I term
  integral_ += error * dt;
  // capping i terms
  i_term_ = std::clamp(gains_.ki * integral_, -gains_.integral_limit, gains_.integral_limit);
  if (gains_.ki != 0.0) {
    integral_ = i_term_ / gains_.ki;
  }

  // D term
  if (has_prev_ && dt > 0.0) {
    d_term_ = gains_.kd * (error - prev_error_) / dt;
  } else {
    d_term_ = 0.0;
  }

  prev_error_ = error;
  has_prev_ = true;

  // calculate final pid value, cap with std::clamp
  double fin = std::clamp(p_term_ + i_term_ + d_term_, -gains_.output_limit, gains_.output_limit);
  return fin;
}

double headingError(double target_deg, double current_deg)
{
  return std::remainder(target_deg - current_deg, 360.0);
}

}  // namespace controller
