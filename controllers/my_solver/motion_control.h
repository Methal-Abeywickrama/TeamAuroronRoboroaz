#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <algorithm>
#include <cmath>
#include <iostream>

// Future Sensor Fusion Integration
#include "sensing.h"
// Note: We will use sensor fusion of IMU and IR sensors in both sides for
// future in this motion controller file.

namespace Motion {

struct PIDConfig {
  double Kp;
  double Ki;
  double Kd;
  double max_output;   // Clamped to this magnitude (e.g., 6.28)
  double min_output;   // Clamped to this magnitude (e.g., -6.28)
  double max_integral; // Anti-windup limit for integral term
  double
      derivative_filter_alpha; // Low-pass filter smoothing factor (0.0 - 1.0)

  PIDConfig()
      : Kp(0), Ki(0), Kd(0), max_output(6.28), min_output(-6.28),
        max_integral(5.0), derivative_filter_alpha(0.1) {}
};

struct PIDResult {
  double output;
  double p_term;
  double i_term;
  double d_term;
};

class PID {
public:
  PID(const PIDConfig &config) : config_(config) { reset(); }

  void reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    prev_derivative_ = 0.0;
    first_run_ = true;
  }

  PIDResult calculate(double target, double current, double dt) {
    if (dt <= 0.0) {
      return {0.0, 0.0, 0.0, 0.0};
    }

    double error = target - current;

    // 1. Proportional Term
    double p_term = config_.Kp * error;

    // 2. Integral Term with Anti-Windup (Clamping)
    integral_ += error * dt;

    // Clamp integral to prevent windup
    if (integral_ > config_.max_integral)
      integral_ = config_.max_integral;
    if (integral_ < -config_.max_integral)
      integral_ = -config_.max_integral;

    double i_term = config_.Ki * integral_;

    // 3. Derivative Term with Low-Pass Filter
    double derivative = 0.0;
    if (!first_run_) {
      double raw_derivative = (error - prev_error_) / dt;
      // Filter: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
      derivative = config_.derivative_filter_alpha * raw_derivative +
                   (1.0 - config_.derivative_filter_alpha) * prev_derivative_;
    } else {
      derivative = 0.0;
      first_run_ = false;
    }

    prev_error_ = error;
    prev_derivative_ = derivative;

    double d_term = config_.Kd * derivative;

    // 4. Total Output with Clamping
    double output = p_term + i_term + d_term;

    if (output > config_.max_output)
      output = config_.max_output;
    if (output < config_.min_output)
      output = config_.min_output;

    return {output, p_term, i_term, d_term};
  }

  // Allow updating config on the fly if needed
  void setConfig(const PIDConfig &config) { config_ = config; }
  PIDConfig getConfig() const { return config_; }

private:
  PIDConfig config_;
  double integral_;
  double prev_error_;
  double prev_derivative_;
  bool first_run_;
};

} // namespace Motion

#endif // MOTION_CONTROL_H
