/*
  MotorControlDemo — RP2040 (ex. Waveshare RP2040 Zero) : PWM HG7881, ADC courant,
  AS5600 en I2C, ISR 10 kHz (courant) + position 2 kHz dans loop().

  Autres cartes : démo stub (sans matériel).

  Ouvre via : Fichier > Exemples > motor_control > MotorControlDemo
*/

#if defined(ARDUINO_ARCH_RP2040)

extern void setup_pico_motor_demo();
extern void loop_pico_motor_demo();

void setup() {
  setup_pico_motor_demo();
}

void loop() {
  loop_pico_motor_demo();
}

#else

#include <motor_control.hpp>

class DummyCurrentSensor final : public motor_control::ICurrentSensor {
public:
  float read_amperes() override { return 0.0F; }
};

class StubHg7881Pins final : public motor_control::IHg7881BridgePins {
public:
  void set_ia_duty(float duty_0_1) override { (void)duty_0_1; }
  void set_ib_duty(float duty_0_1) override { (void)duty_0_1; }
};

DummyCurrentSensor g_current_sensor;
motor_control::CurrentLoopController g_current_loop(g_current_sensor);
StubHg7881Pins g_hg7881_pins;
motor_control::Hg7881MotorDriver g_motor_driver(g_hg7881_pins);

void setup() {
  Serial.begin(115200);
  delay(500);

  g_current_loop.set_sample_time_s(0.001F);
  g_current_loop.set_pi_gains(0.05F, 8.0F);
  g_current_loop.set_output_limits(-1.0F, 1.0F);
  g_current_loop.reset();

  Serial.println(F("motor_control demo (stub — compiler avec une carte RP2040 pour la démo matérielle)"));
}

void loop() {
  g_current_loop.update();
  const float u = g_current_loop.step(0.2F);
  g_motor_driver.set_command(u);

  static uint32_t last_ms;
  const uint32_t now = millis();
  if (now - last_ms >= 200) {
    last_ms = now;
    Serial.println(u, 4);
  }
  delay(1);
}

#endif
