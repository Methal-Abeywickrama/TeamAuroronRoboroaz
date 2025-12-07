#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#define _USE_MATH_DEFINES
#include "sensing.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <webots/Motor.hpp>

namespace Motion {

struct PIDConfig {
  double Kp, Ki, Kd;
  double max_output, min_output;
  double max_integral;
  double derivative_filter_alpha;
  double max_ramp_rate;

  PIDConfig()
      : Kp(0), Ki(0), Kd(0), max_output(6.28), min_output(-6.28),
        max_integral(5.0), derivative_filter_alpha(0.3), max_ramp_rate(15.0) {}
};

struct PIDResult {
  double output, p_term, i_term, d_term;
};

class PID {
private:
  PIDConfig config_;
  double integral_, prev_error_, prev_derivative_, prev_output_;
  bool first_run_;

public:
  PID(const PIDConfig &config) : config_(config) { reset(); }

  void reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    prev_derivative_ = 0.0;
    prev_output_ = 0.0;
    first_run_ = true;
  }

  PIDResult calculate(double target, double current, double dt,
                      bool isAngle = false) {
    if (dt <= 0.0)
      return {0.0, 0.0, 0.0, 0.0};

    double error = target - current;

    // Angle Wrapping: Normalize error to [-PI, PI]
    if (isAngle) {
      while (error > M_PI)
        error -= 2.0 * M_PI;
      while (error < -M_PI)
        error += 2.0 * M_PI;
    }

    double p_term = config_.Kp * error;

    // Integral with anti-windup
    integral_ += error * dt;
    integral_ = std::max(std::min(integral_, config_.max_integral),
                         -config_.max_integral);
    double i_term = config_.Ki * integral_;

    // Filtered derivative
    double derivative = 0.0;
    if (!first_run_) {
      double raw_deriv = (error - prev_error_) / dt;
      derivative = config_.derivative_filter_alpha * raw_deriv +
                   (1.0 - config_.derivative_filter_alpha) * prev_derivative_;
    } else {
      first_run_ = false;
    }
    prev_error_ = error;
    prev_derivative_ = derivative;
    double d_term = config_.Kd * derivative;

    // Sum and apply slew rate limiting
    double raw_output = p_term + i_term + d_term;
    double output = raw_output;

    // Ramp rate limiting
    double max_change = config_.max_ramp_rate * dt;
    output = std::max(std::min(output, prev_output_ + max_change),
                      prev_output_ - max_change);

    // Final clamping
    output = std::max(std::min(output, config_.max_output), config_.min_output);
    prev_output_ = output;

    return {output, p_term, i_term, d_term};
  }

  void setConfig(const PIDConfig &config) { config_ = config; }
  PIDConfig getConfig() const { return config_; }
};

class MotionController {
private:
  webots::Motor *leftMotor, *rightMotor;
  Sensor::Sensing *sensors;
  webots::Robot *robot;
  std::ofstream debugFile;
  PID distPID, anglePID;

  const double WHEEL_RADIUS = 0.02001;
  const double MAX_SPEED = 6.28;

  // Centering parameters
  const double WALL_TARGET = 0.0900;  // Ideal distance from wall
  const double WALL_DEADZONE = 0.015; // ±15mm no correction zone
  const double WALL_MAX_DIST = 0.20;  // Only correct within 20cm

  enum State { IDLE, DRIVING, ROTATING };
  State currentState;

  double targetDistance, targetHeading;
  double startEncLeft, startEncRight;
  double stopPhaseDistance; // When to enter stopping phase
  bool inSoftStopMode;      // Track if we've switched to soft stop

  void configureMotors() {
    if (leftMotor) {
      leftMotor->setPosition(INFINITY);
      leftMotor->setVelocity(0.0);
    }
    if (rightMotor) {
      rightMotor->setPosition(INFINITY);
      rightMotor->setVelocity(0.0);
    }
  }

  void setupDrivePID() {
    PIDConfig cfg;
    cfg.Kp = 12.0; // Reduced from 15.0
    cfg.Ki = 0.0;
    cfg.Kd = 1.2; // Increased damping
    cfg.max_output = MAX_SPEED;
    cfg.min_output = -MAX_SPEED;
    cfg.max_integral = 3.0;
    cfg.derivative_filter_alpha = 0.3; // Better filtering
    cfg.max_ramp_rate = 15.0;          // Smoother acceleration
    distPID.setConfig(cfg);
  }

  void setupAnglePID(bool isRotating = false, bool softStop = false) {
    PIDConfig cfg = anglePID.getConfig(); // Keep current config as base

    if (isRotating) {
      cfg.Kp = 2.0;
      cfg.Ki = 0.0;
      cfg.Kd = 0.8;
      cfg.max_ramp_rate = 15.0;
    } else if (softStop) {
      // Soft stop: gentle corrections
      cfg.Kp = 0.68; // Reduced from 1.2 to prevent crash
      cfg.Ki = 0.01;
      cfg.Kd = 0.05; // Reduced from 0.5
      cfg.max_ramp_rate = 10.0;
    } else {
      // Normal driving: strong corrections
      cfg.Kp = 0.8;  // Increased from 0.8 for tighter control
      cfg.Ki = 0.05; // Reduced from 0.05 to reduce weaving
      cfg.Kd = 0.05;
      cfg.max_ramp_rate = 15.0;
    }

    cfg.max_output = MAX_SPEED;
    cfg.min_output = -MAX_SPEED;
    cfg.max_integral = 2.0;
    cfg.derivative_filter_alpha = 0.2;
    anglePID.setConfig(cfg);
  }

  double normalizeAngle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  // Wall repulsion - gain changes in soft stop
  double calculateWallCentering(double distRemaining, bool softStop) {
    double dLeft = sensors->getDistance(5);
    double dRight = sensors->getDistance(2);

    double correction = 0.0;

    // Lower gain in soft stop mode
    double gain =
        softStop ? 0.5 : 1.0; // REDUCED from 1.5/3.5 to prevent oscillation

    // Left wall repulsion (push right when too close)
    if (dLeft < WALL_TARGET) {
      double intrusion = WALL_TARGET - dLeft;
      if (intrusion > WALL_DEADZONE) {
        correction -=
            intrusion * gain; // FIX: Turn Right (Negative) to avoid Left Wall
      }
    }

    // Right wall repulsion (push left when too close)
    if (dRight < WALL_TARGET) {
      double intrusion = WALL_TARGET - dRight;
      if (intrusion > WALL_DEADZONE) {
        correction +=
            intrusion * gain; // FIX: Turn Left (Positive) to avoid Right Wall
      }
    }

    // Allow stronger correction
    return std::max(std::min(correction, 0.5), -0.5); // REDUCED CLAMP from 2.5
  }

  void logDebugInfo(double dt, double currentDist, double remaining,
                    double currentYaw, double errorYaw, double angleOutput,
                    const PIDResult &angleDebug, double wallCorrection,
                    double leftSpeed, double rightSpeed) {

    if (!debugFile.is_open())
      return;

    debugFile << std::fixed << std::setprecision(4);
    debugFile << "--- TIMESTAMP: " << robot->getTime() << " ---" << std::endl;
    debugFile << "State: "
              << (currentState == DRIVING
                      ? "DRIVING"
                      : (currentState == ROTATING ? "ROTATING" : "IDLE"))
              << (inSoftStopMode ? " (SoftStop)" : "") << std::endl;

    debugFile << std::setprecision(5) << "Dist: " << currentDist
              << " | Rem: " << remaining << std::endl;

    debugFile << std::setprecision(4) << "Yaw: " << currentYaw
              << " | Tgt: " << targetHeading << " | Err: " << errorYaw
              << std::endl;

    PIDConfig angleCfg = anglePID.getConfig();
    debugFile << "AnglePID: Out=" << angleOutput << " (P=" << angleDebug.p_term
              << " I=" << angleDebug.i_term << " D=" << angleDebug.d_term << ")"
              << std::endl;

    debugFile << "PID Config: Kp=" << angleCfg.Kp << " Ki=" << angleCfg.Ki
              << " Kd=" << angleCfg.Kd << std::endl;

    // Extra Debug Data: Raw Sensors
    double rawLeft = sensors->getDistance(5);
    double rawRight = sensors->getDistance(2);
    debugFile << "WallCorr: " << wallCorrection
              << " | RawSensors(L/R): " << rawLeft << " / " << rawRight
              << std::endl;

    debugFile << "Motors: L=" << leftSpeed << " R=" << rightSpeed << std::endl;
    debugFile << "----------------------------------------" << std::endl;
  }

public:
  MotionController(webots::Robot *robot, Sensor::Sensing *s)
      : sensors(s), distPID(PIDConfig()), anglePID(PIDConfig()) {

    leftMotor = robot->getMotor("left wheel motor");
    rightMotor = robot->getMotor("right wheel motor");

    configureMotors();
    setupDrivePID();
    setupAnglePID(false);

    currentState = IDLE;
    targetDistance = targetHeading = 0.0;
    startEncLeft = startEncRight = 0.0;
    stopPhaseDistance = 0.0;
    inSoftStopMode = false;

    this->robot = robot;
    // Open Debug File (Overwrite mode)
    debugFile.open("debug_report.txt", std::ios::out | std::ios::trunc);
    if (debugFile.is_open()) {
      debugFile << "=== STARTING CONTROLLER LOG ===" << std::endl;
    }
  }

  // Destructor to close file
  ~MotionController() {
    if (debugFile.is_open()) {
      debugFile << "=== END OF LOG ===" << std::endl;
      debugFile.close();
    }
  }

  void moveForward(double meters) {
    distPID.reset();
    anglePID.reset();
    setupAnglePID(false, false); // Start in normal driving mode

    if (sensors) {
      startEncLeft = sensors->getLeftEncoder();
      startEncRight = sensors->getRightEncoder();
      targetHeading = sensors->getYaw();
    }

    targetDistance = meters;
    stopPhaseDistance = 0.05; // Fixed 5cm soft stop zone
    inSoftStopMode = false;
    currentState = DRIVING;

    std::cout << "Moving " << meters << "m, heading: " << targetHeading
              << std::endl;
  }

  void turnLeft(int steps = 1) {
    if (!sensors)
      return;

    targetHeading = normalizeAngle(sensors->getYaw() + (steps * M_PI / 2.0));
    currentState = ROTATING;

    anglePID.reset();
    setupAnglePID(true); // Rotation mode

    std::cout << "Turn Left " << steps << " -> " << targetHeading << std::endl;
  }

  void turnRight(int steps = 1) {
    if (!sensors)
      return;

    targetHeading = normalizeAngle(sensors->getYaw() - (steps * M_PI / 2.0));
    currentState = ROTATING;

    anglePID.reset();
    setupAnglePID(true); // Rotation mode

    std::cout << "Turn Right " << steps << " -> " << targetHeading << std::endl;
  }

  void update(double dt) {
    if (currentState == IDLE || !sensors || !leftMotor || !rightMotor)
      return;

    if (currentState == DRIVING) {
      // Get current position
      double currL = sensors->getLeftEncoder();
      double currR = sensors->getRightEncoder();
      double distL = (currL - startEncLeft) * WHEEL_RADIUS;
      double distR = (currR - startEncRight) * WHEEL_RADIUS;
      double currentDist = (distL + distR) / 2.0;
      double remaining = targetDistance - currentDist;

      // Check if we should enter soft stop mode (remaining < 5cm)
      if (!inSoftStopMode && remaining <= stopPhaseDistance && remaining > 0) {
        inSoftStopMode = true;
        // Update PID config WITHOUT resetting errors
        setupAnglePID(false, true);
        std::cout << "Entering soft stop mode at " << currentDist << "m"
                  << std::endl;
      }

      // Distance control
      PIDResult distRes = distPID.calculate(targetDistance, currentDist, dt);
      double baseSpeed = distRes.output;

      // Heading correction with normalized angle error
      double currentYaw = sensors->getYaw();
      PIDResult angleRes =
          anglePID.calculate(targetHeading, currentYaw, dt, true);
      double turnCorrection = angleRes.output;

      // Wall centering
      if (remaining > 0.02) { // Active until very close to goal
        turnCorrection += calculateWallCentering(remaining, inSoftStopMode);
      }

      // Motor mixing
      double leftSpeed =
          std::max(std::min(baseSpeed - turnCorrection, MAX_SPEED), -MAX_SPEED);
      double rightSpeed =
          std::max(std::min(baseSpeed + turnCorrection, MAX_SPEED), -MAX_SPEED);

      leftMotor->setVelocity(leftSpeed);
      rightMotor->setVelocity(rightSpeed);

      // Debug Logging
      double errorYaw = normalizeAngle(targetHeading - currentYaw);
      logDebugInfo(dt, currentDist, remaining, currentYaw, errorYaw,
                   turnCorrection, angleRes, turnCorrection - angleRes.output,
                   leftSpeed, rightSpeed);

      // Completion check
      if (std::abs(remaining) < 0.015 && std::abs(baseSpeed) < 0.5) {
        stop();
        std::cout << "Goal reached. Final dist: " << currentDist << "m"
                  << std::endl;
      }
    }

    if (currentState == ROTATING) {
      double currentYaw = sensors->getYaw();

      PIDResult turnRes =
          anglePID.calculate(targetHeading, currentYaw, dt, true);
      double turnSpeed = turnRes.output;

      leftMotor->setVelocity(-turnSpeed);
      rightMotor->setVelocity(turnSpeed);

      double errorYaw = normalizeAngle(targetHeading - currentYaw);
      if (std::abs(errorYaw) < 0.01 && std::abs(turnSpeed) < 0.3) {
        stop();
        std::cout << "Turn complete. Yaw: " << currentYaw << std::endl;
      }
    }
  }

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