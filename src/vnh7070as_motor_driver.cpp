#include "motor_control/vnh7070as_motor_driver.hpp"

#include <algorithm>
#include <cmath>

namespace motor_control {

Vnh7070asMotorDriver::Vnh7070asMotorDriver(IVnh7070asBridgePins& pins) : pins_(pins) {}

void Vnh7070asMotorDriver::set_command(float normalized) {
  const float u = std::clamp(normalized, -1.0F, 1.0F);
  const float duty = std::fabs(u);
  if (!std::isfinite(duty)) {
    disable_outputs();
    return;
  }
  if (duty <= 0.0F) {
    disable_outputs();
    return;
  }
  if (u > 0.0F) {
    pins_.set_ina(true);
    pins_.set_inb(false);
  } else {
    pins_.set_ina(false);
    pins_.set_inb(true);
  }
  pins_.set_pwm_duty(duty);
}

void Vnh7070asMotorDriver::disable_outputs() {
  pins_.set_ina(false);
  pins_.set_inb(false);
  pins_.set_pwm_duty(0.0F);
}

}  // namespace motor_control
