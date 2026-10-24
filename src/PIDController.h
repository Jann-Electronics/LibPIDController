#pragma once

#include <math.h>
#include <stdint.h>

namespace PID {

/* ---------------------------------------------------------------------------------------------- */
/*                                             Enums                                              */
/* ---------------------------------------------------------------------------------------------- */

// Gain set. Normal is used while asymmetric gains are disabled. With asymmetric gains enabled, Up
// is used while the error is positive (measurement below the setpoint) and Down while it is
// negative; a zero error keeps the current set
enum class GainMode : uint8_t {
  Normal = 0,
  Up     = 1,
  Down   = 2,
};

// Signal the derivative action is computed from
enum class DerivativeMode : uint8_t {
  OnMeasurement, // -d(input)/dt: no output spike when the setpoint changes (default)
  OnError,       // d(error)/dt: textbook form, a setpoint step produces a derivative kick
};

/* ---------------------------------------------------------------------------------------------- */
/*                                             State                                              */
/* ---------------------------------------------------------------------------------------------- */

// Snapshot of the controller internals after the last update(), as returned by getState(). All
// fields come from the same iteration, so prefer it over separate getters when logging or plotting
struct State {
  float setpoint;     // setpoint used in the last iteration
  float input;        // last measurement passed to update() (may be NAN if it was invalid)
  float error;        // setpoint - input
  float proportional; // proportional action
  float integral;     // integral action (accumulated)
  float derivative;   // derivative action (after the N filter)
  float unsaturated;  // proportional + integral + derivative, before the output limits
  float output;       // final output, clamped to [min, max]
  GainMode gain_mode; // gain set used in the last iteration
  bool enabled;       // controller enabled
  bool saturated;     // output clamped to one of the limits
  bool integrating;   // false while the anti-windup holds the integral
  bool in_deadband;   // |error| below the deadband threshold (P and I see a zero error)
  bool stable;        // same as isStable()
};

/* ---------------------------------------------------------------------------------------------- */
/*                                           Controller                                           */
/* ---------------------------------------------------------------------------------------------- */

// Discrete PID controller (Tustin/bilinear discretization) with filtered derivative, output
// limits, clamping anti-windup, deadband, asymmetric gains and stability detection.
//
//   u = Kp·e + Ki·∫e dt + Kd·N·s/(s + N)·(e or -input)
//
// update() must be called once every sample time (setSampleTimeMs()); the controller does not
// measure time itself. The controller starts disabled: call setEnabled(true) after configuring it.
// Not thread-safe: guard it externally if update() and the setters run on different tasks.
class Controller {
  public:
  Controller();

  // Runs one iteration with the latest measurement and returns the new output.
  // While disabled it returns 0. A NAN or infinite measurement is ignored: the internal state is
  // kept and the previous output is returned
  float update(float input);

  // Clears the internal state (integral, derivative history, output and stability counter).
  // Configuration is not changed
  void reset();

  // ----- Enable -----

  // Enabling a disabled controller calls reset(), so it starts from a clean state
  void setEnabled(bool enabled);
  bool isEnabled() const;

  // ----- Setpoint -----
  void setSetpoint(float setpoint);
  float getSetpoint() const;

  // ----- Gains -----

  // Gains of a gain set (Normal by default). Changing Ki at runtime does not bump the output: the
  // integral stores the accumulated Ki·e, not ∫e
  void setGainProportional(float Kp, GainMode mode = GainMode::Normal);
  void setGainIntegral(float Ki, GainMode mode = GainMode::Normal);
  void setGainDerivative(float Kd, GainMode mode = GainMode::Normal);
  void setGains(float Kp, float Ki, float Kd, GainMode mode = GainMode::Normal);

  float getGainProportional(GainMode mode = GainMode::Normal) const;
  float getGainIntegral(GainMode mode = GainMode::Normal) const;
  float getGainDerivative(GainMode mode = GainMode::Normal) const;

  // Derivative filter coefficient N, in rad/s (default 20). The derivative is low-pass filtered
  // with a cutoff at N. Returns false (and changes nothing) if N is not positive or not finite
  bool setDerivativeFilter(float N, GainMode mode = GainMode::Normal);
  float getDerivativeFilter(GainMode mode = GainMode::Normal) const;

  void setDerivativeMode(DerivativeMode mode);
  DerivativeMode getDerivativeMode() const;

  // ----- Sample time -----

  // Period between update() calls (default 10 ms). Returns false (and changes nothing) if 0
  bool setSampleTimeMs(uint32_t sample_time_ms);
  uint32_t getSampleTimeMs() const;
  float getSampleTimeSec() const;

  // ----- Output limits -----

  // Output range (default -100 to 100). Returns false (and changes nothing) if a value is NAN or
  // min >= max
  bool setOutputLimits(float min, float max);
  bool setOutputLimitMin(float min);
  bool setOutputLimitMax(float max);
  float getOutputLimitMin() const;
  float getOutputLimitMax() const;

  // ----- Anti-windup -----

  // Clamping anti-windup: the integral is held while the output is saturated and integrating would
  // push it further into saturation
  void setAntiWindupEnabled(bool enabled);
  bool isAntiWindupEnabled() const;

  // ----- Deadband -----

  // While |error| < threshold, the proportional and integral actions see a zero error: P is 0, the
  // integral is held and the output settles at the integral (plus derivative) value
  void setDeadbandEnabled(bool enabled);
  bool isDeadbandEnabled() const;

  // Returns false (and changes nothing) if the threshold is negative or NAN
  bool setDeadbandThreshold(float threshold);
  float getDeadbandThreshold() const;

  // ----- Asymmetric gains -----

  // Switches between the Up and Down gain sets by the sign of the error. Up and Down start at 0:
  // configure both before enabling. Disabling returns to the Normal set
  void setAsymmetricGainsEnabled(bool enabled);
  bool isAsymmetricGainsEnabled() const;

  // Gain set in use
  GainMode getGainMode() const;

  // ----- Stability detection -----

  // isStable() returns true once |error| <= threshold for `samples` consecutive iterations, and
  // false again as soon as the error leaves the band. Detection is off while either value is 0.
  // setStableErrorThreshold() returns false (and changes nothing) if the threshold is negative or
  // NAN. Both setters restart the count
  bool setStableErrorThreshold(float threshold);
  float getStableErrorThreshold() const;

  void setStableSamples(uint16_t samples);
  uint16_t getStableSamples() const;

  bool isStable() const;

  // ----- Diagnostics -----

  // Snapshot of every internal value of the last iteration (see State)
  State getState() const;

  float getOutput() const;
  float getLastInput() const;

  private:
  struct Gains {
    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;
    float N  = 20.0f;
  };

  static constexpr uint8_t GAIN_MODE_COUNT = 3;

  const Gains& activeGains() const;
  void updateCoefficients();
  void selectGainMode(GainMode mode);

  // Configuration
  Gains _gains[GAIN_MODE_COUNT];
  GainMode _gain_mode             = GainMode::Normal;
  DerivativeMode _derivative_mode = DerivativeMode::OnMeasurement;

  float _setpoint           = 0.0f;
  uint32_t _sample_time_ms  = 10;
  float _T                  = 0.01f;
  float _output_min         = -100.0f;
  float _output_max         = 100.0f;
  float _deadband_threshold = 0.0f;
  float _stable_threshold   = 0.0f;
  uint16_t _stable_samples  = 0;

  bool _enabled                  = false;
  bool _antiwindup_enabled       = false;
  bool _deadband_enabled         = false;
  bool _asymmetric_gains_enabled = false;

  // Discrete coefficients of the active gain set
  float _a = 0.0f; // integral:   Ki·T/2
  float _b = 0.0f; // derivative: 2·Kd·N/(N·T + 2)
  float _c = 0.0f; // derivative: (N·T - 2)/(N·T + 2)

  // State
  bool _first_run          = true; // no previous sample yet: derivative starts at 0
  float _input             = 0.0f;
  float _last_input        = 0.0f; // previous valid measurement (derivative on measurement)
  float _error             = 0.0f;
  float _last_error        = 0.0f; // previous error seen by P/I/D (0 inside the deadband)
  float _proportional      = 0.0f;
  float _integral          = 0.0f;
  float _derivative        = 0.0f;
  float _unsaturated       = 0.0f;
  float _output            = 0.0f;
  bool _integrating        = true;
  bool _in_deadband        = false;
  uint16_t _stable_counter = 0;
};

} // namespace PID
