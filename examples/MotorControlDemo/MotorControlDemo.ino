/*
  MotorControlDemo — exemple minimal utilisant motor_control (CMake + Arduino IDE).

  Prérequis : ce dépôt installé comme bibliothèque "motor_control"
  (voir ../README.md : lien symbolique vers ~/Documents/Arduino/libraries/motor_control).

  Les HAL ci-dessous sont des stubs : remplace-les par ADC / PWM / GPIO réels.
*/

#include <motor_control/motor_control.hpp>

// --- Stubs matériel (à remplacer) -------------------------------------------

class DummyCurrentSensor final : public motor_control::ICurrentSensor {
public:
  float read_amperes() override { return 0.0F; }
};

class StubHg7881Pins final : public motor_control::IHg7881BridgePins {
public:
  void set_ia_duty(float duty_0_1) override {
    (void)duty_0_1;
    // TODO: analogWrite / ledc / timer PWM sur broche IA
  }
  void set_ib_duty(float duty_0_1) override {
    (void)duty_0_1;
    // TODO: broche IB
  }
};

// --- Objets globaux (ordre : capteur → boucle → HAL → driver) --------------

DummyCurrentSensor g_current_sensor;
motor_control::CurrentLoopController g_current_loop(g_current_sensor);
StubHg7881Pins g_hg7881_pins;
motor_control::Hg7881MotorDriver g_motor_driver(g_hg7881_pins);

void setup() {
  Serial.begin(115200);
  delay(800);  // laisse le temps au moniteur série de s’ouvrir (évite blocage sans USB)

  const float dt = 0.001F;  // 1 kHz si tu appelles update/step à cette cadence
  g_current_loop.set_sample_time_s(dt);
  g_current_loop.set_pi_gains(0.05F, 8.0F);
  g_current_loop.set_output_limits(-1.0F, 1.0F);
  g_current_loop.reset();

  Serial.println(F("motor_control demo — boucle courant + driver HG7881 (stub)"));
}

void loop() {
  // Séquence type ISR : acquisition puis calcul
  g_current_loop.update();
  const float u = g_current_loop.step(0.2F);  // consigne 0,2 A (stub mesure = 0)
  g_motor_driver.set_command(u);

  static uint32_t last_ms;
  const uint32_t now = millis();
  if (now - last_ms >= 200) {
    last_ms = now;
    Serial.print(F("sortie PI (normalisée): "));
    Serial.println(u, 4);
  }

  delay(1);  // remplacer par un timer hardware en prod
}
