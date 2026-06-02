#pragma once

namespace motor_control {

/// Mesure d'angle ou de position pour l'asservissement. L'unité doit être la même que la consigne
/// passée à \ref PositionCurrentCascadeController::step (ex. radians pour un axe rotatif, mètres pour linéaire).
class IPositionSensor {
public:
  IPositionSensor() = default;
  IPositionSensor(const IPositionSensor&) = delete;
  IPositionSensor& operator=(const IPositionSensor&) = delete;
  virtual ~IPositionSensor() = default;

  /// Position / angle mesuré (même unité que la consigne de la cascade).
  virtual float read_position() = 0;
};

}  // namespace motor_control
