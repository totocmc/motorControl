#pragma once

namespace motor_control {

/// Abstraction for a DC brushed motor armature current measurement (amperes).
/// Implémentations concrètes (ADC + shunt, driver avec ISEN, etc.) vivent hors de cette lib.
class ICurrentSensor {
public:
  ICurrentSensor() = default;
  ICurrentSensor(const ICurrentSensor&) = delete;
  ICurrentSensor& operator=(const ICurrentSensor&) = delete;
  virtual ~ICurrentSensor() = default;

  /// Courant instantané du moteur, en ampères (A). Signe selon convention torque / sens de rotation.
  virtual float read_amperes() = 0;
};

}  // namespace motor_control
