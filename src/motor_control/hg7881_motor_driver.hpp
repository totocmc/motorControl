#pragma once

#include "motor_control/hg7881_bridge_pins.hpp"
#include "motor_control/motor_pwm_driver.hpp"

namespace motor_control {

/// Pilotage d’un canal HG7881 (ou module équivalent L9110) : avant = IA en PWM, IB à 0 ; arrière = l’inverse.
/// \ref disable_outputs met les deux entrées à 0 (stand-by / roue libre selon module).
class Hg7881MotorDriver final : public IMotorPwmDriver {
public:
  explicit Hg7881MotorDriver(IHg7881BridgePins& pins);

  void set_command(float normalized) override;
  void disable_outputs() override;

private:
  IHg7881BridgePins& pins_;
};

}  // namespace motor_control
