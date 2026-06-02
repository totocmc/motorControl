#pragma once

#include <cmath>

namespace motor_control {

/// Paramètres statiques d'un moteur DC brossé (couple ∝ courant : τ ≈ Kt * I).
struct DcBrushedMotorParams {
  /// Constante de couple en N·m/A (souvent proche de ke en V/(rad/s) en unités SI).
  float kt_nm_per_a{0.0F};
  float max_current_a{0.0F};  ///< Limite thermique / driver (0 = pas de limite appliquée ici)

  [[nodiscard]] float torque_to_current_setpoint(float torque_nm) const {
    if (kt_nm_per_a <= 0.0F) {
      return 0.0F;
    }
    float i = torque_nm / kt_nm_per_a;
    if (max_current_a > 0.0F) {
      const float m = std::fabs(max_current_a);
      if (i > m) {
        i = m;
      }
      if (i < -m) {
        i = -m;
      }
    }
    return i;
  }

  [[nodiscard]] float current_to_torque_estimate(float current_a) const {
    return kt_nm_per_a * current_a;
  }
};

}  // namespace motor_control
