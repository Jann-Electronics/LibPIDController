/**
 * Basic PID temperature control
 *
 * The PID computes the heating power needed to hold a target temperature (setpoint).
 *
 * Hardware:
 *   - LM35 temperature sensor (10 mV/°C) on TEMP_SENSOR_PIN
 *   - Heater driven through a MOSFET on HEATER_PIN (PWM)
 *
 * Output is CSV, ready for the Arduino Serial Plotter.
 */

#include <Arduino.h>

#include "PIDController.h"

// Pins (adjust according to hardware)
constexpr uint8_t TEMP_SENSOR_PIN = 1;
constexpr uint8_t HEATER_PIN      = 9;

// PID parameters
constexpr float KP            = 2.0f;
constexpr float KI            = 0.5f;
constexpr float KD            = 0.1f;
constexpr float TARGET_TEMP_C = 50.0f;
constexpr uint32_t SAMPLE_MS  = 100;

PID::Controller pid;

// LM35: 10 mV per °C
float readTemperature() { return analogReadMilliVolts(TEMP_SENSOR_PIN) / 10.0f; }

void setup() {
  Serial.begin(115200);
  pinMode(HEATER_PIN, OUTPUT);

  pid.setSetpoint(TARGET_TEMP_C);
  pid.setGains(KP, KI, KD);
  pid.setSampleTimeMs(SAMPLE_MS);

  // 0 % (off) to 100 % (full power)
  pid.setOutputLimits(0.0f, 100.0f);

  // Stop the integral from winding up while the heater is saturated
  pid.setAntiWindupEnabled(true);

  pid.setEnabled(true);

  Serial.println("Setpoint(C),Temperature(C),Output(%)");
}

void loop() {
  // The PID assumes a fixed period: schedule with millis() instead of delay(), so the time spent
  // reading and printing does not stretch the sample time
  static uint32_t last_ms = millis();
  if (millis() - last_ms < SAMPLE_MS) return;
  last_ms += SAMPLE_MS;

  float temperature = readTemperature();
  float output      = pid.update(temperature);

  // 0-100 % to 8-bit PWM
  analogWrite(HEATER_PIN, static_cast<uint8_t>(output * 255.0f / 100.0f));

  Serial.printf("%.2f,%.2f,%.2f\n", TARGET_TEMP_C, temperature, output);
}
