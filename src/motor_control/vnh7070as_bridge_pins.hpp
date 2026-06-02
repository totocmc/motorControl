#pragma once

namespace motor_control {

/// Port matériel pour VNH7070AS : entrées logiques INA / INB et broche PWM (modulation bas-côté).
/// SEL0 / CS restent gérés hors de cette interface (ex. capteur de courant MultiSense).
class IVnh7070asBridgePins {
public:
  IVnh7070asBridgePins() = default;
  IVnh7070asBridgePins(const IVnh7070asBridgePins&) = delete;
  IVnh7070asBridgePins& operator=(const IVnh7070asBridgePins&) = delete;
  virtual ~IVnh7070asBridgePins() = default;

  virtual void set_ina(bool high) = 0;
  virtual void set_inb(bool high) = 0;
  /// 0 = PWM à l’arrêt (bas-côtés coupés) ; 1 = plein cycle utile côté pont (selon timer MCU).
  virtual void set_pwm_duty(float duty_0_1) = 0;
};

}  // namespace motor_control
