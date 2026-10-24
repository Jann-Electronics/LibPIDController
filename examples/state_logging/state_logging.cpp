/**
 * Logging the controller internals with getState()
 *
 * Runs the PID against a simulated first-order thermal plant, so no hardware is needed. Every
 * sample prints the full controller state as CSV (Serial Plotter friendly): error, P, I and D
 * actions, output and the status flags.
 *
 * Halfway through, the setpoint steps from 50 to 70 °C. With the derivative on the measurement
 * (default) the D action does not spike; switch to DerivativeMode::OnError to see the difference.
 */

#include <Arduino.h>

#include "PIDController.h"

constexpr uint32_t SAMPLE_MS = 100;

PID::Controller pid;

// Simulated plant: tau·dT/dt = -(T - ambient) + gain·power
struct ThermalPlant {
  float temperature   = 25.0f;
  const float ambient = 25.0f;
  const float tau_s   = 30.0f;
  const float gain_c  = 0.8f; // °C per % of power, at steady state

  float step(float power, float dt_s) {
    temperature += dt_s * (-(temperature - ambient) + gain_c * power) / tau_s;
    return temperature;
  }
} plant;

void setup() {
  Serial.begin(115200);

  // Wait up to 3 s for the USB serial monitor, so the board also runs without a PC attached
  while (!Serial && millis() < 3000) {
  }

  pid.setSetpoint(50.0f);
  pid.setGains(4.0f, 0.15f, 2.0f);
  pid.setDerivativeFilter(10.0f);
  pid.setSampleTimeMs(SAMPLE_MS);
  pid.setOutputLimits(0.0f, 100.0f);
  pid.setAntiWindupEnabled(true);

  // isStable() after 3 s within ±0.5 °C
  pid.setStableErrorThreshold(0.5f);
  pid.setStableSamples(3000 / SAMPLE_MS);

  pid.setEnabled(true);
}

void loop() {
  static uint32_t last_ms = millis();
  static uint32_t samples = 0;

  if (millis() - last_ms < SAMPLE_MS) return;
  last_ms += SAMPLE_MS;

  if (++samples == 1200) pid.setSetpoint(70.0f); // after 2 minutes

  pid.update(plant.temperature);
  plant.step(pid.getOutput(), SAMPLE_MS / 1000.0f);

  const PID::State s = pid.getState();
  Serial.printf("setpoint=%.2f,input=%.2f,error=%.2f,P=%.2f,I=%.2f,D=%.2f,output=%.2f,saturated=%d,"
                "integrating=%d,stable=%d\n",
    s.setpoint,
    s.input,
    s.error,
    s.proportional,
    s.integral,
    s.derivative,
    s.output,
    s.saturated,
    s.integrating,
    s.stable);
}
