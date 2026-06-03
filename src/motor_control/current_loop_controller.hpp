#pragma once

#include "current_sensor.hpp"

namespace motor_control {

/// Boucle de courant (PI) pour moteur DC : sortie typiquement consigne de tension normalisée ou à ramener en duty-cycle côté PWM.
class CurrentLoopController {
public:
  /// \p sensor doit rester valide pendant toute la durée de vie du contrôleur (référence non possédante).
  explicit CurrentLoopController(ICurrentSensor& sensor);

  void set_pi_gains(float kp, float ki);
  void set_output_limits(float min_u, float max_u);
  void set_integral_limit(float abs_max_integral);  ///< 0 = pas de limite sur l'intégrale

  /// Période d'échantillonnage fixe (ISR timer) pour \ref step sans \p dt_s. Ignoré si ≤ 0.
  void set_sample_time_s(float dt_s);
  [[nodiscard]] float sample_time_s() const { return sample_time_s_; }

  void reset();

  /// Échantillonne le courant (ADC / driver). Appeler avant \ref step pour séparer acquisition / calcul (ISR typique).
  void update();

  /// Mesure le courant : dernière valeur issue de \ref update si disponible, sinon lecture capteur (comportement historique).
  /// \p dt_s > 0 requis.
  /// \return commande (ex. [-1, 1] ou volts selon limites fixées).
  float step(float current_reference_a, float dt_s);

  /// Même logique que \ref step(float, float) avec \ref set_sample_time_s (sinon équivalent à dt = 0 → pas d'intégration).
  float step(float current_reference_a) { return step(current_reference_a, sample_time_s_); }

  [[nodiscard]] float last_output() const { return last_u_; }
  [[nodiscard]] float integral_term() const { return integral_; }
  /// Dernier courant mesuré utilisé ou acquis par \ref update (A), si fini.
  [[nodiscard]] float last_measured_current_a() const { return last_i_meas_; }

private:
  ICurrentSensor& sensor_;
  float kp_{0.0F};
  float ki_{0.0F};
  float u_min_{-1.0F};
  float u_max_{1.0F};
  float integral_limit_{0.0F};
  float integral_{0.0F};
  float last_u_{0.0F};
  float sample_time_s_{0.0F};
  float i_meas_cached_{0.0F};
  bool use_i_meas_cache_{false};
  float last_i_meas_{0.0F};
};

}  // namespace motor_control
