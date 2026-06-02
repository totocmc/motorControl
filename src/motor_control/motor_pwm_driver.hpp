#pragma once

namespace motor_control {

/// Abstraction du pilotage PWM (pont en H, driver brushé, etc.) pour un moteur DC.
/// Les implémentations concrètes configurent la fréquence / résolution côté matériel.
class IMotorPwmDriver {
public:
  IMotorPwmDriver() = default;
  IMotorPwmDriver(const IMotorPwmDriver&) = delete;
  IMotorPwmDriver& operator=(const IMotorPwmDriver&) = delete;
  virtual ~IMotorPwmDriver() = default;

  /// Consigne normalisée typiquement dans [-1, 1] : sens et amplitude (rapport cyclique signé ou équivalent).
  /// Les valeurs hors plage peuvent être saturées par l’implémentation.
  virtual void set_command(float normalized) = 0;

  /// Met le driver en état sûr : PWM off, sorties haute impédance ou frein selon le matériel.
  virtual void disable_outputs() = 0;
};

}  // namespace motor_control
