#pragma once

#include "motor_pwm_driver.hpp"
#include "vnh7070as_bridge_pins.hpp"

namespace motor_control {

/// Pilotage \ref IMotorPwmDriver pour VNH7070AS (VIPower M0-7) : sens via INA/INB, amplitude via PWM.
/// Convention positive : INA=1, INB=0 ; négative : l’inverse. Arrêt : INA=INB=PWM=0 (stand-by datasheet).
class Vnh7070asMotorDriver final : public IMotorPwmDriver {
public:
  explicit Vnh7070asMotorDriver(IVnh7070asBridgePins& pins);

  void set_command(float normalized) override;
  void disable_outputs() override;

private:
  IVnh7070asBridgePins& pins_;
};

}  // namespace motor_control
