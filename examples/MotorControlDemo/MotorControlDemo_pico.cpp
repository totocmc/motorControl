#if defined(ARDUINO_ARCH_RP2040)

#include <Arduino.h>
#include <Wire.h>
#include <atomic>
#include <cmath>

#include <motor_control.hpp>
#include "pico/time.h"

// -----------------------------------------------------------------------------
// Brochage (adapter au câblage — Waveshare RP2040 Zero)
// -----------------------------------------------------------------------------
static constexpr uint8_t kPinPwmIa = 16;
static constexpr uint8_t kPinPwmIb = 17;
static constexpr uint8_t kPinAdcCurrent = A0;  // GPIO26 — tension image courant (V)
static constexpr uint8_t kPinSda = 4;
static constexpr uint8_t kPinScl = 5;

static constexpr uint8_t kAs5600Addr = 0x36;
static constexpr float kTwoPi = 6.283185307179586476925286766559f;

// Fréquences
static constexpr float kCurrentDtS = 1.0e-4f;   // 10 kHz
static constexpr int kCurrentPeriodUs = 100;
static constexpr float kPositionDtS = 5.0e-4f;  // 2 kHz
static constexpr uint32_t kPositionPeriodUs = 500;

// Calibration courant : I = (V - Voffset) / VperAmp
static constexpr float kCurrentVoltsPerAmp = 0.185f;
static constexpr float kCurrentOffsetVolts = 1.65f;

// Résolutions alignées sur setup_pico_motor_demo() (cœur Arduino-rp2040 : pas de getter
// analogReadResolution(), pas d’analogReadMilliVolts — analogRead + échelle explicite).
static constexpr int kAnalogReadBits = 12;
static constexpr int kAnalogWriteBits = 10;
static constexpr float kAdcVrefVolts = 3.3f;

static float g_angle_ref_rad = 0.0f;
static float g_pos_integral = 0.0f;
static constexpr float g_pos_kp = 3.0f;
static constexpr float g_pos_ki = 25.0f;
static constexpr float g_pos_integral_limit = 8.0f;
static constexpr float g_i_ref_min_a = -2.0f;
static constexpr float g_i_ref_max_a = 2.0f;

static std::atomic<float> g_i_ref_a{0.0f};
static std::atomic<float> g_telemetry_u{0.0f};
static std::atomic<float> g_telemetry_i_meas{0.0f};
static std::atomic<uint32_t> g_isr_count{0};
static std::atomic<bool> g_motor_armed{true};

static repeating_timer_t g_current_timer{};

static float smallest_angle_error(float reference_rad, float measured_rad) {
  return std::atan2(std::sin(reference_rad - measured_rad), std::cos(reference_rad - measured_rad));
}

template <typename WireT>
auto wire_begin_with_pins_impl(WireT& w, uint8_t sda, uint8_t scl, int) -> decltype((void)w.begin(sda, scl)) {
  w.begin(sda, scl);
}

template <typename WireT>
void wire_begin_with_pins_impl(WireT& w, uint8_t /*sda*/, uint8_t /*scl*/, long) {
  w.begin();
}

static void wire_begin_for_demo() {
  wire_begin_with_pins_impl(Wire, kPinSda, kPinScl, 0);
}

static uint16_t pwm_max_value() {
  return static_cast<uint16_t>((1u << static_cast<unsigned>(kAnalogWriteBits)) - 1u);
}

static void write_pwm_duty01(uint8_t pin, float duty_0_1) {
  duty_0_1 = std::max(0.0f, std::min(duty_0_1, 1.0f));
  const uint16_t mx = pwm_max_value();
  analogWrite(pin, static_cast<int>(duty_0_1 * static_cast<float>(mx)));
}

class PicoHg7881Pins final : public motor_control::IHg7881BridgePins {
public:
  void set_ia_duty(float duty_0_1) override { write_pwm_duty01(kPinPwmIa, duty_0_1); }
  void set_ib_duty(float duty_0_1) override { write_pwm_duty01(kPinPwmIb, duty_0_1); }
};

class PicoAdcCurrentSensor final : public motor_control::ICurrentSensor {
public:
  explicit PicoAdcCurrentSensor(uint8_t pin) : pin_(pin) {}

  float read_amperes() override {
    const int raw = analogRead(pin_);
    const float denom_counts = static_cast<float>((1 << kAnalogReadBits) - 1);
    const float v = (static_cast<float>(raw) / denom_counts) * kAdcVrefVolts;
    const float denom = std::max(1.0e-6f, kCurrentVoltsPerAmp);
    return (v - kCurrentOffsetVolts) / denom;
  }

private:
  uint8_t pin_;
};

class As5600AngleRad final {
public:
  bool probe() {
    Wire.beginTransmission(kAs5600Addr);
    Wire.write(0x00);
    return Wire.endTransmission() == 0;
  }

  float read_radians() {
    const uint16_t raw = read_raw12();
    return static_cast<float>(raw) * (kTwoPi / 4096.0f);
  }

private:
  uint16_t read_raw12() {
    Wire.beginTransmission(kAs5600Addr);
    Wire.write(0x0C);
    if (Wire.endTransmission(false) != 0) {
      return 0;
    }
    if (Wire.requestFrom(static_cast<int>(kAs5600Addr), 2) != 2) {
      return 0;
    }
    const uint8_t msb = static_cast<uint8_t>(Wire.read());
    const uint8_t lsb = static_cast<uint8_t>(Wire.read());
    const uint16_t ang = (static_cast<uint16_t>(msb) << 8) | static_cast<uint16_t>(lsb);
    return static_cast<uint16_t>(ang & 0x0FFFu);
  }
};

static PicoAdcCurrentSensor g_i_sense(kPinAdcCurrent);
static motor_control::CurrentLoopController g_current_loop(g_i_sense);
static PicoHg7881Pins g_hg7881_pins;
static motor_control::Hg7881MotorDriver g_motor_driver(g_hg7881_pins);
static As5600AngleRad g_as5600;

static void run_position_outer_2khz(float position_reference, float measured_rad) {
  if (!std::isfinite(measured_rad) || !std::isfinite(position_reference)) {
    const float hold = std::clamp(g_i_ref_a.load(std::memory_order_relaxed), g_i_ref_min_a, g_i_ref_max_a);
    g_i_ref_a.store(hold, std::memory_order_relaxed);
    return;
  }

  const float err = smallest_angle_error(position_reference, measured_rad);
  if (!std::isfinite(err)) {
    const float hold = std::clamp(g_i_ref_a.load(std::memory_order_relaxed), g_i_ref_min_a, g_i_ref_max_a);
    g_i_ref_a.store(hold, std::memory_order_relaxed);
    return;
  }

  float proposed_integral = g_pos_integral + err * kPositionDtS;
  proposed_integral = std::clamp(proposed_integral, -g_pos_integral_limit, g_pos_integral_limit);

  float i_ref = g_pos_kp * err + g_pos_ki * proposed_integral;
  i_ref = std::clamp(i_ref, g_i_ref_min_a, g_i_ref_max_a);

  const bool saturated_high = (i_ref >= g_i_ref_max_a) && (err > 0.0f);
  const bool saturated_low = (i_ref <= g_i_ref_min_a) && (err < 0.0f);
  if (!saturated_high && !saturated_low) {
    g_pos_integral = proposed_integral;
    g_pos_integral = std::clamp(g_pos_integral, -g_pos_integral_limit, g_pos_integral_limit);
  }

  g_i_ref_a.store(i_ref, std::memory_order_relaxed);
}

static bool current_timer_isr(struct repeating_timer* /*t*/) {
  g_isr_count.fetch_add(1u, std::memory_order_relaxed);

  if (!g_motor_armed.load(std::memory_order_relaxed)) {
    g_motor_driver.disable_outputs();
    g_telemetry_u.store(0.0f, std::memory_order_relaxed);
    return true;
  }

  g_current_loop.update();
  const float i_sp = g_i_ref_a.load(std::memory_order_relaxed);
  const float u = g_current_loop.step(i_sp, kCurrentDtS);
  g_motor_driver.set_command(u);

  g_telemetry_i_meas.store(g_current_loop.last_measured_current_a(), std::memory_order_relaxed);
  g_telemetry_u.store(u, std::memory_order_relaxed);
  return true;
}

void setup_pico_motor_demo() {
  Serial.begin(115200);
  delay(300);

  analogWriteResolution(10);
  analogWriteFreq(20000);
  analogReadResolution(12);

  pinMode(kPinPwmIa, OUTPUT);
  pinMode(kPinPwmIb, OUTPUT);
  analogWrite(kPinPwmIa, 0);
  analogWrite(kPinPwmIb, 0);

  wire_begin_for_demo();
  Wire.setClock(400000);

  if (!g_as5600.probe()) {
    Serial.println(F("AS5600 introuvable sur I2C — vérifie SDA/SCL et l’aimant."));
  }

  g_angle_ref_rad = g_as5600.read_radians();
  g_pos_integral = 0.0f;
  g_i_ref_a.store(0.0f, std::memory_order_relaxed);

  g_current_loop.set_sample_time_s(kCurrentDtS);
  g_current_loop.set_pi_gains(0.12f, 18.0f);
  g_current_loop.set_output_limits(-1.0f, 1.0f);
  g_current_loop.reset();

  if (!add_repeating_timer_us(-kCurrentPeriodUs, current_timer_isr, nullptr, &g_current_timer)) {
    Serial.println(F("Échec add_repeating_timer_us (10 kHz)."));
  }

  Serial.println(F("motor_control RP2040 : courant 10 kHz (ISR), position 2 kHz (loop), HG7881 + AS5600."));
  Serial.println(F("Touches : z=aligner ref, h=coupure PWM + reset boucle courant, r=réarme PWM, s=ref +0.2 rad."));
}

void loop_pico_motor_demo() {
  static uint32_t last_pos_us = 0;
  const uint32_t now_us = micros();
  if (static_cast<uint32_t>(now_us - last_pos_us) >= kPositionPeriodUs) {
    last_pos_us = now_us;
    const float theta = g_as5600.read_radians();
    run_position_outer_2khz(g_angle_ref_rad, theta);
  }

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == 'z' || c == 'Z') {
      g_angle_ref_rad = g_as5600.read_radians();
      g_pos_integral = 0.0f;
      Serial.println(F("Référence angle = mesure actuelle."));
    } else if (c == 'h' || c == 'H') {
      noInterrupts();
      g_motor_armed.store(false, std::memory_order_relaxed);
      g_i_ref_a.store(0.0f, std::memory_order_relaxed);
      g_current_loop.reset();
      g_motor_driver.disable_outputs();
      interrupts();
      Serial.println(F("Arrêt : PWM coupé, boucle courant réinitialisée."));
    } else if (c == 'r' || c == 'R') {
      g_motor_armed.store(true, std::memory_order_relaxed);
      Serial.println(F("PWM réarmé (ISR 10 kHz)."));
    } else if (c == 's' || c == 'S') {
      g_angle_ref_rad += 0.2f;
      Serial.println(F("Référence +0.2 rad."));
    }
  }

  static uint32_t last_print_ms = 0;
  const uint32_t tms = millis();
  if (tms - last_print_ms >= 200) {
    last_print_ms = tms;
    const uint32_t ticks = g_isr_count.exchange(0u);
    Serial.print(F("u="));
    Serial.print(g_telemetry_u.load(std::memory_order_relaxed), 4);
    Serial.print(F(" i_A="));
    Serial.print(g_telemetry_i_meas.load(std::memory_order_relaxed), 4);
    Serial.print(F(" isr_200ms="));
    Serial.print(ticks);
    Serial.println(F(" (attendu ~2000 si 10 kHz)"));
  }
}

#endif  // ARDUINO_ARCH_RP2040
