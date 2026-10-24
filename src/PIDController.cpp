#include "PIDController.h"

namespace PID {

namespace {
constexpr uint8_t index(GainMode mode) { return static_cast<uint8_t>(mode); }
} // namespace

/* ---------------------------------------------------------------------------------------------- */
/*                                          Constructor                                           */
/* ---------------------------------------------------------------------------------------------- */
Controller::Controller() { updateCoefficients(); }

/* ---------------------------------------------------------------------------------------------- */
/*                                             Update                                             */
/* ---------------------------------------------------------------------------------------------- */
float Controller::update(float input) {
  _input = input;

  if (!_enabled) return _output; // reset() by setEnabled(false), so the output is 0

  // An invalid measurement would turn the integral into NAN for good: skip the iteration
  if (!isfinite(input)) return _output;

  _error = _setpoint - input;

  // Stability detection uses the raw error, not the deadband one
  if (_stable_threshold > 0.0f && _stable_samples > 0) {
    if (fabsf(_error) <= _stable_threshold) {
      if (_stable_counter < _stable_samples) _stable_counter++;
    } else {
      _stable_counter = 0;
    }
  }

  // Positive error: Up gains. Negative error: Down gains. Zero error: keep the current set
  if (_asymmetric_gains_enabled) {
    if (_error > 0.0f) {
      selectGainMode(GainMode::Up);
    } else if (_error < 0.0f) {
      selectGainMode(GainMode::Down);
    }
  }

  // Inside the deadband P and I see a zero error
  _in_deadband  = _deadband_enabled && fabsf(_error) < _deadband_threshold;
  const float e = _in_deadband ? 0.0f : _error;

  _proportional = activeGains().Kp * e;

  // Filtered derivative: D[k] = b·Δ - c·D[k-1]. Without a previous sample Δ would be a step from
  // 0 to the first value, so the first iteration after a reset uses Δ = 0
  float delta = 0.0f;
  if (!_first_run) {
    delta = (_derivative_mode == DerivativeMode::OnMeasurement) ? -(input - _last_input)
                                                                : (e - _last_error);
  }
  _first_run  = false;
  _derivative = _b * delta - _c * _derivative;

  // Trapezoidal integral: I[k] = I[k-1] + a·(e[k] + e[k-1])
  const float increment = _a * (e + _last_error);

  // Clamping anti-windup: hold the integral if the output is saturated and the increment would
  // push it further in the same direction
  _integrating = true;
  if (_antiwindup_enabled) {
    const float candidate = _proportional + _integral + _derivative;
    if ((candidate >= _output_max && increment > 0.0f) ||
        (candidate <= _output_min && increment < 0.0f)) {
      _integrating = false;
    }
  }
  if (_integrating) _integral += increment;

  _unsaturated = _proportional + _integral + _derivative;

  if (_unsaturated > _output_max) {
    _output = _output_max;
  } else if (_unsaturated < _output_min) {
    _output = _output_min;
  } else {
    _output = _unsaturated;
  }

  _last_error = e;
  _last_input = input;

  return _output;
}

void Controller::reset() {
  _first_run      = true;
  _last_input     = 0.0f;
  _error          = 0.0f;
  _last_error     = 0.0f;
  _proportional   = 0.0f;
  _integral       = 0.0f;
  _derivative     = 0.0f;
  _unsaturated    = 0.0f;
  _output         = 0.0f;
  _integrating    = true;
  _in_deadband    = false;
  _stable_counter = 0;
}

/* ---------------------------------------------------------------------------------------------- */
/*                                             Enable                                             */
/* ---------------------------------------------------------------------------------------------- */
void Controller::setEnabled(bool enabled) {
  if (enabled != _enabled) reset();
  _enabled = enabled;
}

bool Controller::isEnabled() const { return _enabled; }

/* ---------------------------------------------------------------------------------------------- */
/*                                            Setpoint                                            */
/* ---------------------------------------------------------------------------------------------- */
void Controller::setSetpoint(float setpoint) { _setpoint = setpoint; }

float Controller::getSetpoint() const { return _setpoint; }

/* ---------------------------------------------------------------------------------------------- */
/*                                             Gains                                              */
/* ---------------------------------------------------------------------------------------------- */
void Controller::setGainProportional(float Kp, GainMode mode) { _gains[index(mode)].Kp = Kp; }

void Controller::setGainIntegral(float Ki, GainMode mode) {
  _gains[index(mode)].Ki = Ki;
  if (mode == _gain_mode) updateCoefficients();
}

void Controller::setGainDerivative(float Kd, GainMode mode) {
  _gains[index(mode)].Kd = Kd;
  if (mode == _gain_mode) updateCoefficients();
}

void Controller::setGains(float Kp, float Ki, float Kd, GainMode mode) {
  Gains& g = _gains[index(mode)];
  g.Kp     = Kp;
  g.Ki     = Ki;
  g.Kd     = Kd;
  if (mode == _gain_mode) updateCoefficients();
}

float Controller::getGainProportional(GainMode mode) const { return _gains[index(mode)].Kp; }

float Controller::getGainIntegral(GainMode mode) const { return _gains[index(mode)].Ki; }

float Controller::getGainDerivative(GainMode mode) const { return _gains[index(mode)].Kd; }

bool Controller::setDerivativeFilter(float N, GainMode mode) {
  if (!isfinite(N) || N <= 0.0f) return false;
  _gains[index(mode)].N = N;
  if (mode == _gain_mode) updateCoefficients();
  return true;
}

float Controller::getDerivativeFilter(GainMode mode) const { return _gains[index(mode)].N; }

void Controller::setDerivativeMode(DerivativeMode mode) { _derivative_mode = mode; }

DerivativeMode Controller::getDerivativeMode() const { return _derivative_mode; }

/* ---------------------------------------------------------------------------------------------- */
/*                                          Sample time                                           */
/* ---------------------------------------------------------------------------------------------- */
bool Controller::setSampleTimeMs(uint32_t sample_time_ms) {
  if (sample_time_ms == 0) return false;
  _sample_time_ms = sample_time_ms;
  _T              = static_cast<float>(sample_time_ms) / 1000.0f;
  updateCoefficients();
  return true;
}

uint32_t Controller::getSampleTimeMs() const { return _sample_time_ms; }

float Controller::getSampleTimeSec() const { return _T; }

/* ---------------------------------------------------------------------------------------------- */
/*                                         Output limits                                          */
/* ---------------------------------------------------------------------------------------------- */
bool Controller::setOutputLimits(float min, float max) {
  if (isnan(min) || isnan(max) || min >= max) return false;
  _output_min = min;
  _output_max = max;
  return true;
}

bool Controller::setOutputLimitMin(float min) { return setOutputLimits(min, _output_max); }

bool Controller::setOutputLimitMax(float max) { return setOutputLimits(_output_min, max); }

float Controller::getOutputLimitMin() const { return _output_min; }

float Controller::getOutputLimitMax() const { return _output_max; }

/* ---------------------------------------------------------------------------------------------- */
/*                                          Anti-windup                                           */
/* ---------------------------------------------------------------------------------------------- */
void Controller::setAntiWindupEnabled(bool enabled) { _antiwindup_enabled = enabled; }

bool Controller::isAntiWindupEnabled() const { return _antiwindup_enabled; }

/* ---------------------------------------------------------------------------------------------- */
/*                                            Deadband                                            */
/* ---------------------------------------------------------------------------------------------- */
void Controller::setDeadbandEnabled(bool enabled) { _deadband_enabled = enabled; }

bool Controller::isDeadbandEnabled() const { return _deadband_enabled; }

bool Controller::setDeadbandThreshold(float threshold) {
  if (isnan(threshold) || threshold < 0.0f) return false;
  _deadband_threshold = threshold;
  return true;
}

float Controller::getDeadbandThreshold() const { return _deadband_threshold; }

/* ---------------------------------------------------------------------------------------------- */
/*                                        Asymmetric gains                                        */
/* ---------------------------------------------------------------------------------------------- */
void Controller::setAsymmetricGainsEnabled(bool enabled) {
  _asymmetric_gains_enabled = enabled;
  if (!enabled) selectGainMode(GainMode::Normal);
}

bool Controller::isAsymmetricGainsEnabled() const { return _asymmetric_gains_enabled; }

GainMode Controller::getGainMode() const { return _gain_mode; }

/* ---------------------------------------------------------------------------------------------- */
/*                                      Stability detection                                       */
/* ---------------------------------------------------------------------------------------------- */
bool Controller::setStableErrorThreshold(float threshold) {
  if (isnan(threshold) || threshold < 0.0f) return false;
  _stable_threshold = threshold;
  _stable_counter   = 0;
  return true;
}

float Controller::getStableErrorThreshold() const { return _stable_threshold; }

void Controller::setStableSamples(uint16_t samples) {
  _stable_samples = samples;
  _stable_counter = 0;
}

uint16_t Controller::getStableSamples() const { return _stable_samples; }

bool Controller::isStable() const {
  if (_stable_threshold <= 0.0f || _stable_samples == 0) return false;
  return _stable_counter >= _stable_samples;
}

/* ---------------------------------------------------------------------------------------------- */
/*                                          Diagnostics                                           */
/* ---------------------------------------------------------------------------------------------- */
State Controller::getState() const {
  State state;
  state.setpoint     = _setpoint;
  state.input        = _input;
  state.error        = _error;
  state.proportional = _proportional;
  state.integral     = _integral;
  state.derivative   = _derivative;
  state.unsaturated  = _unsaturated;
  state.output       = _output;
  state.gain_mode    = _gain_mode;
  state.enabled      = _enabled;
  state.saturated    = _enabled && (_unsaturated > _output_max || _unsaturated < _output_min);
  state.integrating  = _integrating;
  state.in_deadband  = _in_deadband;
  state.stable       = isStable();
  return state;
}

float Controller::getOutput() const { return _output; }

float Controller::getLastInput() const { return _input; }

/* ---------------------------------------------------------------------------------------------- */
/*                                            Private                                             */
/* ---------------------------------------------------------------------------------------------- */
const Controller::Gains& Controller::activeGains() const { return _gains[index(_gain_mode)]; }

// Tustin discretization of Ki/s and Kd·N·s/(s + N)
void Controller::updateCoefficients() {
  const Gains& g = activeGains();
  const float NT = g.N * _T;
  _a             = g.Ki * _T * 0.5f;
  _b             = (2.0f * g.Kd * g.N) / (NT + 2.0f);
  _c             = (NT - 2.0f) / (NT + 2.0f);
}

void Controller::selectGainMode(GainMode mode) {
  if (mode == _gain_mode) return;
  _gain_mode = mode;
  updateCoefficients();
}

} // namespace PID
