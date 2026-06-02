#pragma once

namespace motor_control {

/// Port matériel pour un demi-pont HG7881 / L9110 : broches IA et IB (PWM 0…1 ou niveau selon implémentation).
class IHg7881BridgePins {
public:
  IHg7881BridgePins() = default;
  IHg7881BridgePins(const IHg7881BridgePins&) = delete;
  IHg7881BridgePins& operator=(const IHg7881BridgePins&) = delete;
  virtual ~IHg7881BridgePins() = default;

  /// 0 = sortie forcée basse ; 1 = PWM plein rail (ou continu « haut » selon ton HAL).
  virtual void set_ia_duty(float duty_0_1) = 0;
  virtual void set_ib_duty(float duty_0_1) = 0;
};

}  // namespace motor_control
