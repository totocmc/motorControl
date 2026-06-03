#include <motor_control.hpp>

namespace motor_control {

namespace {
float smallest_angle_error(float reference_rad, float measured_rad) {
  return std::atan2(std::sin(reference_rad - measured_rad), std::cos(reference_rad - measured_rad));
}
}  // namespace

PositionCurrentCascadeController::PositionCurrentCascadeController(IPositionSensor& position,
                                                                   CurrentLoopController& current)
    : position_(position), current_(current) {}

void PositionCurrentCascadeController::set_control_mode(CascadeControlMode mode) {
  mode_ = mode;
}

void PositionCurrentCascadeController::set_position_pi_gains(float kp_a_per_rad, float ki_a_per_rad_s) {
  pos_kp_ = kp_a_per_rad;
  pos_ki_ = ki_a_per_rad_s;
}

void PositionCurrentCascadeController::set_position_integral_limit(float abs_max_integral) {
  pos_integral_limit_ = abs_max_integral >= 0.0F ? abs_max_integral : 0.0F;
}

void PositionCurrentCascadeController::set_current_setpoint_limits(float min_a, float max_a) {
  i_ref_min_ = std::min(min_a, max_a);
  i_ref_max_ = std::max(min_a, max_a);
}

void PositionCurrentCascadeController::set_wrap_angle_error(bool wrap) {
  wrap_angle_error_ = wrap;
}

void PositionCurrentCascadeController::reset_position_integrator() {
  pos_integral_ = 0.0F;
}

void PositionCurrentCascadeController::set_sample_time_s(float dt_s) {
  const float t = (dt_s > 0.0F && std::isfinite(dt_s)) ? dt_s : 0.0F;
  sample_time_s_ = t;
  current_.set_sample_time_s(t);
}

void PositionCurrentCascadeController::update() {
  current_.update();
  if (mode_ == CascadeControlMode::PositionServo) {
    const float p = position_.read_position();
    pos_meas_cached_ = p;
    use_pos_meas_cache_ = std::isfinite(p);
    if (use_pos_meas_cache_) {
      last_pos_meas_ = p;
    }
  } else {
    use_pos_meas_cache_ = false;
  }
}

float PositionCurrentCascadeController::step(float position_reference) {
  float dt = sample_time_s_;
  if (!(dt > 0.0F)) {
    dt = current_.sample_time_s();
  }
  return step(position_reference, dt);
}

float PositionCurrentCascadeController::position_error(float reference, float measured) const {
  if (wrap_angle_error_) {
    return smallest_angle_error(reference, measured);
  }
  return reference - measured;
}

void PositionCurrentCascadeController::enter_position_servo() {
  mode_ = CascadeControlMode::PositionServo;
}

void PositionCurrentCascadeController::enter_current_hold(float hold_current_a) {
  hold_current_a_ = hold_current_a;
  mode_ = CascadeControlMode::CurrentHold;
}

void PositionCurrentCascadeController::enter_current_hold_using_last_command() {
  hold_current_a_ = last_i_ref_;
  mode_ = CascadeControlMode::CurrentHold;
}

float PositionCurrentCascadeController::step(float position_reference, float dt_s) {
  if (!(dt_s > 0.0F) || !std::isfinite(dt_s)) {
    return current_.last_output();
  }

  float i_ref = 0.0F;

  if (mode_ == CascadeControlMode::CurrentHold) {
    use_pos_meas_cache_ = false;
    i_ref = hold_current_a_;
    if (!std::isfinite(i_ref)) {
      i_ref = 0.0F;
    }
    i_ref = std::clamp(i_ref, i_ref_min_, i_ref_max_);
    last_i_ref_ = i_ref;
    return current_.step(i_ref, dt_s);
  }

  float pos_meas = 0.0F;
  if (use_pos_meas_cache_) {
    pos_meas = pos_meas_cached_;
    use_pos_meas_cache_ = false;
  } else {
    pos_meas = position_.read_position();
  }

  if (!std::isfinite(pos_meas) || !std::isfinite(position_reference)) {
    last_i_ref_ = std::clamp(last_i_ref_, i_ref_min_, i_ref_max_);
    return current_.step(last_i_ref_, dt_s);
  }
  last_pos_meas_ = pos_meas;

  const float err = position_error(position_reference, pos_meas);
  if (!std::isfinite(err)) {
    last_i_ref_ = std::clamp(last_i_ref_, i_ref_min_, i_ref_max_);
    return current_.step(last_i_ref_, dt_s);
  }

  float proposed_integral = pos_integral_ + err * dt_s;
  if (pos_integral_limit_ > 0.0F) {
    const float lim = pos_integral_limit_;
    proposed_integral = std::clamp(proposed_integral, -lim, lim);
  }

  i_ref = pos_kp_ * err + pos_ki_ * proposed_integral;
  i_ref = std::clamp(i_ref, i_ref_min_, i_ref_max_);

  const bool saturated_high = (i_ref >= i_ref_max_) && (err > 0.0F);
  const bool saturated_low = (i_ref <= i_ref_min_) && (err < 0.0F);
  if (!saturated_high && !saturated_low) {
    pos_integral_ = proposed_integral;
    if (pos_integral_limit_ > 0.0F) {
      const float lim = pos_integral_limit_;
      pos_integral_ = std::clamp(pos_integral_, -lim, lim);
    }
  }

  last_i_ref_ = i_ref;
  return current_.step(i_ref, dt_s);
}

}  // namespace motor_control
