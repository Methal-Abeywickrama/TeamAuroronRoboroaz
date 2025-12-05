#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <cmath>
#include <iostream>
#include <webots/Motor.hpp>

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

// -------------------------------------------------------------------------
// Motion Controller Class (Layer 2 & 3 Integration)
// -------------------------------------------------------------------------
// This class bridges the raw PID logic (Layer 1) with the robot's hardware.
// It manages state (Idle vs Driving) and converts high-level commands
// (like "move forward 1 meter") into motor velocity commands.
class MotionController {
private:
  // --- 1. HARDWARE ---
  webots::Motor *leftMotor;
  webots::Motor *rightMotor;
  // Pointer to the Sensing module for access to Encoders and IMU
  Sensing *sensors;

  // --- 2. THE BRAIN ---
  // PID Controller for Distance (Drive Control)
  PID distPID;

  // --- 3. PHYSICAL CONSTANTS (The Calibration Targets) ---
  // Wheel Radius: Critical for Odometry (Rad -> Meters conversion)
  // E-puck standard is approx 20.5mm or 0.0205m.
  double WHEEL_RADIUS = 0.02001;

  // --- 4. STATE MEMORY ---
  enum State { IDLE, DRIVING };
  State currentState;

  double targetDistance; // how far we WANT to go (Meters)
  double startEncLeft;   // Encoder reading when we started
  double startEncRight;  // Encoder reading when we started

public:
  // Constructor
  // Initializes motors in Velocity Mode and configures the PID.
  MotionController(webots::Robot *robot, Sensing *s)
      : sensors(s),
        // Initialize PID with a default config first.
        // We apply the specific tuning in the body below to respect strict
        // struct rules.
        distPID(PIDConfig()) {
    // 1. Get Motor Devices
    leftMotor = robot->getMotor("left wheel motor");
    rightMotor = robot->getMotor("right wheel motor");

    // 2. Configure Motors for Velocity Control
    // Setting position to INFINITY tells Webots to ignore position limits
    // and allow continuous rotation controlled by setVelocity().
    if (leftMotor) {
      leftMotor->setPosition(INFINITY);
      leftMotor->setVelocity(0.0);
    }
    if (rightMotor) {
      rightMotor->setPosition(INFINITY);
      rightMotor->setVelocity(0.0);
    }

    // 3. Configure PID Tuning
    // Create a custom configuration for the Drive PID
    PIDConfig driveConfig;
    driveConfig.Kp = 15.0; // Aggressive Proportional term for snappy response
    driveConfig.Ki =
        0.0; // No Integral term (avoids overshoot for simple moves)
    driveConfig.Kd = 0.5;          // Derivative term to dampen oscillations
    driveConfig.max_output = 6.28; // Clamp to max motor speed
    driveConfig.min_output = -6.28;
    driveConfig.max_integral = 5.0;
    driveConfig.derivative_filter_alpha = 0.1;

    distPID.setConfig(driveConfig);

    currentState = IDLE;
    targetDistance = 0.0;
    startEncLeft = 0.0;
    startEncRight = 0.0;
  }

  // Call this ONCE to start the action
  void moveForward(double meters) {
    // 1. Reset the Brain
    distPID.reset();

    // 2. Take the Snapshot (Relative Odometry Zero)
    if (sensors) {
      startEncLeft = sensors->getLeftEncoder();
      startEncRight = sensors->getRightEncoder();
    }

    // 3. Set Targets
    targetDistance = meters;
    currentState = DRIVING;

    std::cout << "CMD: Moving Forward " << meters << "m" << std::endl;
  }
  // -------------------------------------------------------------------------
  // UPDATE LOOP
  // -------------------------------------------------------------------------

  // It handles the feedback control loop: calculating error -> PID -> Motor
  // Output.
  void update(double dt) {
    // 0. Safety Check: Don't run if devices are missing or logic is idle.
    if (currentState == IDLE || !sensors || !leftMotor || !rightMotor)
      return;

    // --- DRIVING STATE: PID Distance Control ---
    if (currentState == DRIVING) {
      // 1. Get Current Sensor Data (Odometry Read)
      double currL = sensors->getLeftEncoder();
      double currR = sensors->getRightEncoder();

      // 2. Calculate Distance Traveled (Relative to Start)
      // Odometry Formula: ArcLength = Angle(rad) * Radius
      double distL = (currL - startEncLeft) * WHEEL_RADIUS;
      double distR = (currR - startEncRight) * WHEEL_RADIUS;

      // Center Distance = Average of Left and Right frames
      double currentAvgDist = (distL + distR) / 2.0;

      // 3. Ask PID for Required Speed
      // Target: Desired Distance (meters)
      // Current: Actual Distance (meters)
      // Result: Velocity Command (rad/s) for the motors
      PIDResult result = distPID.calculate(targetDistance, currentAvgDist, dt);
      double speed = result.output;

      // 4. Apply Command to Motors
      leftMotor->setVelocity(speed);
      rightMotor->setVelocity(speed);

      // 5. Completion Check (Setpoint Reached?)
      // We consider it "Done" if:
      // a. Error is within tolerance (2mm)
      // b. Robot has effectively stopped (Speed < 0.1 rad/s)
      double error = std::abs(targetDistance - currentAvgDist);
      if (error < 0.002 && std::abs(speed) < 0.1) {
        stop();
        std::cout << "TARGET REACHED. Final Error: " << error << "m"
                  << std::endl;
      }
    }
  }

  // -------------------------------------------------------------------------
  // HELPERS
  // -------------------------------------------------------------------------

  // Emergency/Goal Stop: Zeroes velocity and resets state.
  void stop() {
    if (leftMotor)
      leftMotor->setVelocity(0.0);
    if (rightMotor)
      rightMotor->setVelocity(0.0);
    currentState = IDLE;
  }

  bool isBusy() { return currentState != IDLE; }
};

} // namespace Motion

#endif // MOTION_CONTROL_H
