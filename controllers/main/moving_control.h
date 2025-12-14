#ifndef MOVING_CONTROL_H
#define MOVING_CONTROL_H

#define _USE_MATH_DEFINES
#include "sensing.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <webots/Motor.hpp>

namespace Motion {

// ============================================================================
// MAZE CONSTANTS (E-puck in 0.25m tile maze)
// ============================================================================
struct MazeConfig {
  static constexpr double TILE_SIZE = 0.25;
  static constexpr double ROBOT_WIDTH = 0.074;
  static constexpr double ROBOT_RADIUS = 0.037;
  static constexpr double WALL_CLEARANCE = 0.088;
  static constexpr double TURN_90_RAD = 1.5708;
  static constexpr double WHEEL_RADIUS = 0.02001;
  static constexpr double AXLE_LENGTH = 0.052;
  static constexpr double MAX_SPEED = 6.28;
  static constexpr double INITIAL_HEADING = 1.5709; // North facing start
};

// ============================================================================
// PID TUNING PARAMETERS
// ============================================================================
struct PIDParams {
  static constexpr double DRIVE_KP = 4.0;
  static constexpr double DRIVE_KI = 0.0;
  static constexpr double DRIVE_KD = 1.0;

  static constexpr double ANGLE_KP = 1.5;
  static constexpr double ANGLE_KI = 0.0;
  static constexpr double ANGLE_KD = 0.2;
  static constexpr double ANGLE_MAX_CORRECT = 1.5;

  static constexpr double ROT_KP = 1.5;
  static constexpr double ROT_KI = 0.0;
  static constexpr double ROT_KD = 0.4;

  static constexpr double WALL_GAIN = 3.0;
  static constexpr double WALL_MAX_CORRECT = 0.8;
};

// ============================================================================
// ROBUST PID CONTROLLER
// ============================================================================
class PID {
private:
  double kp_, ki_, kd_;
  double integral_ = 0.0;
  double prevError_ = 0.0;
  double prevDerivative_ = 0.0;
  double prevOutput_ = 0.0;
  bool firstRun_ = true;
  double maxOutput_ = MazeConfig::MAX_SPEED;
  double maxIntegral_ = 3.0;
  double filterAlpha_ = 0.2;
  double maxRamp_ = 20.0;

public:
  void configure(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void reset() {
    integral_ = 0.0;
    prevError_ = 0.0;
    prevDerivative_ = 0.0;
    prevOutput_ = 0.0;
    firstRun_ = true;
  }

  double compute(double error, double dt) {
    if (dt <= 0.0001)
      return prevOutput_;

    double pOut = kp_ * error;

    integral_ += error * dt;
    integral_ = std::max(-maxIntegral_, std::min(maxIntegral_, integral_));
    double iOut = ki_ * integral_;

    double derivative = 0.0;
    if (!firstRun_) {
      double rawDeriv = (error - prevError_) / dt;
      derivative =
          filterAlpha_ * rawDeriv + (1.0 - filterAlpha_) * prevDerivative_;
    }
    firstRun_ = false;
    prevError_ = error;
    prevDerivative_ = derivative;
    double dOut = kd_ * derivative;

    double output = pOut + iOut + dOut;

    double maxChange = maxRamp_ * dt;
    output = std::max(prevOutput_ - maxChange,
                      std::min(prevOutput_ + maxChange, output));
    output = std::max(-maxOutput_, std::min(maxOutput_, output));
    prevOutput_ = output;

    return output;
  }
};

// ============================================================================
// MAIN MOVING CONTROLLER
// ============================================================================
class MovingController {
public:
  enum class State { Idle, Driving, Rotating };
  enum class DriveMode { Normal, FastCorridor };

private:
  webots::Robot *robot_;
  webots::Motor *leftMotor_;
  webots::Motor *rightMotor_;
  Sensor::Sensing *sensors_;
  std::ofstream debugFile_;

  PID distPID_;
  PID anglePID_;
  PID rotPID_;

  State state_ = State::Idle;
  DriveMode driveMode_ = DriveMode::Normal;

  double targetDist_ = 0.0;
  double targetHeading_ = 0.0;
  double startEncL_ = 0.0;
  double startEncR_ = 0.0;

  double fusedHeading_ = MazeConfig::INITIAL_HEADING;
  double odomHeading_ = MazeConfig::INITIAL_HEADING;

  double lastEncL_ = 0.0;
  double lastEncR_ = 0.0;
  bool headingInitialized_ = false;
  static constexpr double FUSION_ALPHA = 0.1;

  double normalizeAngle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  void setMotors(double left, double right) {
    double maxSpd = MazeConfig::MAX_SPEED;
    if (leftMotor_)
      leftMotor_->setVelocity(std::max(-maxSpd, std::min(maxSpd, left)));
    if (rightMotor_)
      rightMotor_->setVelocity(std::max(-maxSpd, std::min(maxSpd, right)));
  }

  void updateFusedHeading(double dt) {
    if (!sensors_)
      return;

    double encL = sensors_->getLeftEncoder();
    double encR = sensors_->getRightEncoder();

    if (!headingInitialized_) {
      odomHeading_ = MazeConfig::INITIAL_HEADING;
      fusedHeading_ = MazeConfig::INITIAL_HEADING;
      lastEncL_ = encL;
      lastEncR_ = encR;
      headingInitialized_ = true;
      return;
    }

    // Pure odometry heading (gyro too noisy in Webots)
    double dL = (encL - lastEncL_) * MazeConfig::WHEEL_RADIUS;
    double dR = (encR - lastEncR_) * MazeConfig::WHEEL_RADIUS;
    double deltaOdom = (dR - dL) / MazeConfig::AXLE_LENGTH;

    lastEncL_ = encL;
    lastEncR_ = encR;

    odomHeading_ = normalizeAngle(odomHeading_ + deltaOdom);
    fusedHeading_ = odomHeading_; // Use pure odometry
  }

  bool inCorridor() {
    if (!sensors_)
      return false;
    double dL = sensors_->getDistance(5);
    double dR = sensors_->getDistance(2);
    return (dL < 0.15 && dR < 0.15);
  }

  double frontClearance() {
    if (!sensors_)
      return 2.0;
    return std::min(sensors_->getDistance(0), sensors_->getDistance(7));
  }

  double getWallCorrection() {
    if (!sensors_)
      return 0.0;

    double dL = sensors_->getDistance(5);
    double dR = sensors_->getDistance(2);
    double correction = 0.0;

    if (dL < 0.15 && dR < 0.15) {
      double error = (dL - dR) / 2.0;
      correction = error * PIDParams::WALL_GAIN;
    } else if (dL < MazeConfig::WALL_CLEARANCE) {
      correction = -(MazeConfig::WALL_CLEARANCE - dL) * PIDParams::WALL_GAIN;
    } else if (dR < MazeConfig::WALL_CLEARANCE) {
      correction = (MazeConfig::WALL_CLEARANCE - dR) * PIDParams::WALL_GAIN;
    }

    return std::max(-PIDParams::WALL_MAX_CORRECT,
                    std::min(PIDParams::WALL_MAX_CORRECT, correction));
  }

  double computeBaseSpeed(double remaining) {
    double speed = MazeConfig::MAX_SPEED;

    if (inCorridor() && frontClearance() > 0.30 && remaining > 0.10) {
      driveMode_ = DriveMode::FastCorridor;
      speed = MazeConfig::MAX_SPEED;
    } else {
      driveMode_ = DriveMode::Normal;
      speed = 6.0; // 2x speed (was 3.0)
    }

    // Deceleration zones (2x faster)
    if (remaining < 0.10)
      speed = std::min(speed, 4.0);
    if (remaining < 0.05)
      speed = std::min(speed, 2.0);
    if (remaining < 0.02)
      speed = std::min(speed, 1.0);

    return speed;
  }

  void logDebug(double dist, double rem, double yaw, double err, double turn,
                double wall, double spd, double leftSpd, double rightSpd) {
    if (!debugFile_.is_open())
      return;

    debugFile_ << std::fixed << std::setprecision(4);
    debugFile_ << "--- TIMESTAMP: " << robot_->getTime() << " ---\n";
    debugFile_ << "State: "
               << (state_ == State::Driving ? "DRIVING" : "ROTATING");
    debugFile_ << " | Mode: "
               << (driveMode_ == DriveMode::FastCorridor ? "FAST" : "NORM")
               << "\n";

    debugFile_ << std::setprecision(5);
    debugFile_ << "Dist: " << dist << " | Rem: " << rem
               << " | Target: " << targetDist_ << "\n";

    debugFile_ << std::setprecision(4);
    debugFile_ << "Yaw: " << yaw << " | Tgt: " << targetHeading_
               << " | Err: " << err << "\n";
    debugFile_ << "OdomYaw: " << odomHeading_
               << " | GyroYaw: " << sensors_->getYaw() << "\n";
    debugFile_ << "TurnCorr: " << turn << " | WallCorr: " << wall << "\n";
    debugFile_ << "Sensors(L/R/F0/F7): " << sensors_->getDistance(5) << " / "
               << sensors_->getDistance(2) << " / " << sensors_->getDistance(0)
               << " / " << sensors_->getDistance(7) << "\n";
    debugFile_ << "Speed: " << spd << " | Motors(L/R): " << leftSpd << " / "
               << rightSpd << "\n";
    debugFile_ << "----------------------------------------\n";
  }

public:
  MovingController(webots::Robot *r, Sensor::Sensing *s)
      : robot_(r), sensors_(s) {
    leftMotor_ = robot_->getMotor("left wheel motor");
    rightMotor_ = robot_->getMotor("right wheel motor");

    if (leftMotor_) {
      leftMotor_->setPosition(INFINITY);
      leftMotor_->setVelocity(0.0);
    }
    if (rightMotor_) {
      rightMotor_->setPosition(INFINITY);
      rightMotor_->setVelocity(0.0);
    }

    distPID_.configure(PIDParams::DRIVE_KP, PIDParams::DRIVE_KI,
                       PIDParams::DRIVE_KD);
    anglePID_.configure(PIDParams::ANGLE_KP, PIDParams::ANGLE_KI,
                        PIDParams::ANGLE_KD);
    rotPID_.configure(PIDParams::ROT_KP, PIDParams::ROT_KI, PIDParams::ROT_KD);

    debugFile_.open("debug_report.txt", std::ios::out | std::ios::trunc);
    if (debugFile_.is_open()) {
      debugFile_ << "=== MOVING CONTROLLER DEBUG REPORT ===\n";
      debugFile_ << "Initial Heading: " << MazeConfig::INITIAL_HEADING
                 << " rad\n";
      debugFile_ << "Tile Size: " << MazeConfig::TILE_SIZE << "m\n";
      debugFile_ << "Fusion Alpha: " << FUSION_ALPHA << "\n";
      debugFile_ << "========================================\n";
    }
  }

  ~MovingController() {
    if (debugFile_.is_open())
      debugFile_.close();
  }

  void moveForward(double tiles) {
    moveForwardMeters(tiles * MazeConfig::TILE_SIZE);
  }

  void moveForwardMeters(double meters) {
    state_ = State::Driving;
    targetDist_ = meters;

    if (sensors_) {
      startEncL_ = sensors_->getLeftEncoder();
      startEncR_ = sensors_->getRightEncoder();
      targetHeading_ = fusedHeading_;
    }

    distPID_.reset();
    anglePID_.reset();

    std::cout << "MOVE: " << meters << "m @ heading " << targetHeading_
              << std::endl;
  }

  void turnLeft() { rotate(MazeConfig::TURN_90_RAD); }
  void turnRight() { rotate(-MazeConfig::TURN_90_RAD); }

  void rotate(double radians) {
    state_ = State::Rotating;
    targetHeading_ = normalizeAngle(fusedHeading_ + radians);
    rotPID_.reset();
    std::cout << "ROTATE: " << radians << " rad -> target " << targetHeading_
              << std::endl;
  }

  void stop() {
    state_ = State::Idle;
    setMotors(0.0, 0.0);
  }

  bool isBusy() const { return state_ != State::Idle; }
  State getState() const { return state_; }

  void update(double dt) {
    if (state_ == State::Idle || !sensors_)
      return;

    updateFusedHeading(dt);

    if (state_ == State::Driving) {
      double encL = sensors_->getLeftEncoder();
      double encR = sensors_->getRightEncoder();
      double dL = (encL - startEncL_) * MazeConfig::WHEEL_RADIUS;
      double dR = (encR - startEncR_) * MazeConfig::WHEEL_RADIUS;
      double currentDist = (dL + dR) / 2.0;
      double remaining = targetDist_ - currentDist;

      double baseSpeed = computeBaseSpeed(remaining);
      double distOutput = distPID_.compute(remaining, dt);
      double speedCmd = std::min(baseSpeed, std::abs(distOutput));
      if (distOutput < 0)
        speedCmd = -speedCmd;

      double headingError = normalizeAngle(targetHeading_ - fusedHeading_);
      double turnCorrection = anglePID_.compute(headingError, dt);

      // CLAMP turn correction
      turnCorrection =
          std::max(-PIDParams::ANGLE_MAX_CORRECT,
                   std::min(PIDParams::ANGLE_MAX_CORRECT, turnCorrection));

      double wallCorrection = 0.0;
      if (remaining > 0.03) {
        wallCorrection = getWallCorrection();
      }

      double totalTurn = turnCorrection + wallCorrection;
      double leftSpeed = speedCmd - totalTurn;
      double rightSpeed = speedCmd + totalTurn;
      setMotors(leftSpeed, rightSpeed);

      logDebug(currentDist, remaining, fusedHeading_, headingError,
               turnCorrection, wallCorrection, speedCmd, leftSpeed, rightSpeed);

      if (std::abs(remaining) < 0.005 && std::abs(speedCmd) < 0.2) {
        stop();
        std::cout << "ARRIVED: dist=" << currentDist << "m" << std::endl;
      }
    } else if (state_ == State::Rotating) {
      double headingError = normalizeAngle(targetHeading_ - fusedHeading_);
      double turnSpeed = rotPID_.compute(headingError, dt);

      setMotors(-turnSpeed, turnSpeed);

      // Tighter tolerance for precise turns (0.005 rad = ~0.3 degrees)
      if (std::abs(headingError) < 0.005 && std::abs(turnSpeed) < 0.1) {
        stop();
        std::cout << "TURN COMPLETE: yaw=" << fusedHeading_ << std::endl;
      }
    }
  }
};

} // namespace Motion

#endif // MOVING_CONTROL_H
