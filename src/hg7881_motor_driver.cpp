#include "motor_control/hg7881_motor_driver.hpp"

#include <algorithm>
#include <cmath>

namespace motor_control {

Hg7881MotorDriver::Hg7881MotorDriver(IHg7881BridgePins& pins) : pins_(pins) {}

void Hg7881MotorDriver::set_command(float normalized) {
  const float u = std::clamp(normalized, -1.0F, 1.0F);
  const float duty = std::fabs(u);
  if (!std::isfinite(duty)) {
    pins_.set_ia_duty(0.0F);
    pins_.set_ib_duty(0.0F);
    return;
  }
  if (u >= 0.0F) {
    pins_.set_ia_duty(duty);
    pins_.set_ib_duty(0.0F);
  } else {
    pins_.set_ia_duty(0.0F);
    pins_.set_ib_duty(duty);
  }
}

void Hg7881MotorDriver::disable_outputs() {
  pins_.set_ia_duty(0.0F);
  pins_.set_ib_duty(0.0F);
}

}  // namespace motor_control
