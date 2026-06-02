#include "motor_control/current_loop_controller.hpp"

#include <algorithm>
#include <cmath>

namespace motor_control {

CurrentLoopController::CurrentLoopController(ICurrentSensor& sensor) : sensor_(sensor) {}

void CurrentLoopController::set_pi_gains(float kp, float ki) {
  kp_ = kp;
  ki_ = ki;
}

void CurrentLoopController::set_output_limits(float min_u, float max_u) {
  u_min_ = std::min(min_u, max_u);
  u_max_ = std::max(min_u, max_u);
}

void CurrentLoopController::set_integral_limit(float abs_max_integral) {
  integral_limit_ = abs_max_integral >= 0.0F ? abs_max_integral : 0.0F;
}

void CurrentLoopController::set_sample_time_s(float dt_s) {
  sample_time_s_ = (dt_s > 0.0F && std::isfinite(dt_s)) ? dt_s : 0.0F;
}

void CurrentLoopController::reset() {
  integral_ = 0.0F;
  last_u_ = 0.0F;
  use_i_meas_cache_ = false;
}

void CurrentLoopController::update() {
  const float i = sensor_.read_amperes();
  i_meas_cached_ = i;
  use_i_meas_cache_ = std::isfinite(i);
  if (use_i_meas_cache_) {
    last_i_meas_ = i;
  }
}

float CurrentLoopController::step(float current_reference_a, float dt_s) {
  if (!(dt_s > 0.0F) || !std::isfinite(dt_s)) {
    return last_u_;
  }

  float i_meas = 0.0F;
  if (use_i_meas_cache_) {
    i_meas = i_meas_cached_;
    use_i_meas_cache_ = false;
  } else {
    i_meas = sensor_.read_amperes();
  }

  if (!std::isfinite(i_meas)) {
    return last_u_;
  }
  last_i_meas_ = i_meas;

  const float err = current_reference_a - i_meas;
  if (!std::isfinite(err)) {
    return last_u_;
  }

  float proposed_integral = integral_ + err * dt_s;
  if (integral_limit_ > 0.0F) {
    const float lim = integral_limit_;
    proposed_integral = std::clamp(proposed_integral, -lim, lim);
  }

  float u = kp_ * err + ki_ * proposed_integral;
  u = std::clamp(u, u_min_, u_max_);

  // Anti-windup : n'intègre que si la sortie ne sature pas dans le sens de l'erreur
  const bool saturated_high = (u >= u_max_) && (err > 0.0F);
  const bool saturated_low = (u <= u_min_) && (err < 0.0F);
  if (!saturated_high && !saturated_low) {
    integral_ = proposed_integral;
    if (integral_limit_ > 0.0F) {
      const float lim = integral_limit_;
      integral_ = std::clamp(integral_, -lim, lim);
    }
  }

  last_u_ = u;
  return last_u_;
}

}  // namespace motor_control
