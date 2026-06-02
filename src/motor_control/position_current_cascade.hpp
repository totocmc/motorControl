#pragma once

#include "motor_control/current_loop_controller.hpp"
#include "motor_control/position_sensor.hpp"

namespace motor_control {

enum class CascadeControlMode {
  /// Boucle position (PI) → consigne courant → \ref CurrentLoopController.
  PositionServo,
  /// Maintien : consigne courant fixe (ex. contre gravité), sans asservissement position.
  CurrentHold,
};

/// Cascade position → courant pour moteur DC brossé. Le courant interne reste toujours actif.
class PositionCurrentCascadeController {
public:
  /// \p position et \p current doivent rester valides pendant toute la durée de vie du contrôleur.
  PositionCurrentCascadeController(IPositionSensor& position, CurrentLoopController& current);

  void set_control_mode(CascadeControlMode mode);
  [[nodiscard]] CascadeControlMode control_mode() const { return mode_; }

  /// PI sur l'erreur de position (sortie = consigne courant, A).
  void set_position_pi_gains(float kp_a_per_rad, float ki_a_per_rad_s);
  void set_position_integral_limit(float abs_max_integral);  ///< 0 = pas de limite

  /// Limite la consigne courant envoyée à la boucle interne (protection moteur / driver).
  void set_current_setpoint_limits(float min_a, float max_a);

  /// Si vrai, l'erreur de position est ramenée dans ]-π, π] (pratique pour un axe en radians).
  void set_wrap_angle_error(bool wrap);

  void reset_position_integrator();

  /// Période fixe (ISR) pour \ref step sans \p dt_s ; propage aussi à la boucle courant interne.
  void set_sample_time_s(float dt_s);
  [[nodiscard]] float sample_time_s() const { return sample_time_s_; }

  /// Échantillonne courant (via la boucle interne) et position. En mode maintien, seul le courant est échantillonné.
  void update();

  /// Mode position : \p position_reference dans la même unité que \ref IPositionSensor::read_position.
  /// Mode maintien : argument ignoré ; utilise le courant de maintien configuré.
  /// \return sortie de la boucle courant (même sémantique que \ref CurrentLoopController::step).
  float step(float position_reference, float dt_s);

  /// Utilise \ref set_sample_time_s, ou à défaut \ref CurrentLoopController::sample_time_s sur la boucle interne.
  float step(float position_reference);

  void enter_position_servo();
  /// Fixe le mode maintien et la consigne courant (A).
  void enter_current_hold(float hold_current_a);
  /// Passe en maintien avec le dernier courant de consigne calculé en mode position.
  void enter_current_hold_using_last_command();

  [[nodiscard]] float hold_current_setpoint() const { return hold_current_a_; }
  [[nodiscard]] float last_current_setpoint() const { return last_i_ref_; }
  [[nodiscard]] float position_integral_term() const { return pos_integral_; }
  [[nodiscard]] float last_measured_position() const { return last_pos_meas_; }

private:
  [[nodiscard]] float position_error(float reference, float measured) const;

  IPositionSensor& position_;
  CurrentLoopController& current_;

  CascadeControlMode mode_{CascadeControlMode::PositionServo};
  float hold_current_a_{0.0F};

  float pos_kp_{0.0F};
  float pos_ki_{0.0F};
  float pos_integral_{0.0F};
  float pos_integral_limit_{0.0F};
  float i_ref_min_{-1.0F};
  float i_ref_max_{1.0F};
  bool wrap_angle_error_{false};

  float last_i_ref_{0.0F};
  float sample_time_s_{0.0F};
  float pos_meas_cached_{0.0F};
  bool use_pos_meas_cache_{false};
  float last_pos_meas_{0.0F};
};

}  // namespace motor_control
