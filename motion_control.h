#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

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

  // Drift compensation: radians of counterclockwise correction per meter
  // Tune this to eliminate systematic clockwise drift during FORWARD movement
  static constexpr double DRIFT_PER_METER = 0.00406;

  // Turn slip compensation: wheel slip during in-place rotation
  // Wheels report more rotation than actually occurs
  // Factor < 1 scales down deltaOdom (0.913 = ~8.7% slip based on 0.137 rad
  // error / 1.5708 rad turn) Tune: if turn undershoots, decrease; if
  // overshoots, increase
  static constexpr double TURN_SLIP_FACTOR = 0.913;
};

// ============================================================================
// PID TUNING PARAMETERS
// ============================================================================
struct PIDParams {
  // Distance PID - higher KP for faster acceleration, higher KD to prevent
  // overshoot
  static constexpr double DRIVE_KP = 15.0; // Was 4.0 - now faster
  static constexpr double DRIVE_KI = 0.0;
  static constexpr double DRIVE_KD =
      3.0; // Was 1.0 - increased to prevent overshoot

  // Heading correction during driving
  static constexpr double ANGLE_KP = 2.5; // Increased for better correction
  static constexpr double ANGLE_KI =
      0.1; // Add integral to eliminate steady-state
  static constexpr double ANGLE_KD = 0.3; // Slightly more damping
  static constexpr double ANGLE_MAX_CORRECT = 1.5;

  // End-of-drive fine alignment (corrects accumulated drift)
  // Note: At 10ms timestep, encoder quantization is ~0.015 rad, so tolerance
  // must be >= this
  static constexpr double DRIVE_FINE_TOLERANCE =
      0.0200; // 0.02 rad = ~1.15 deg (above encoder quantization)
  static constexpr double DRIVE_SETTLE_TIME =
      0.048; // 48ms settle time (timestep-independent)
  static constexpr double DRIVE_MAX_ALIGN_TIME =
      1.0; // Max 1 second for fine alignment

  // Coarse turn phase (fast)
  static constexpr double ROT_KP = 1.5;
  static constexpr double ROT_KI = 0.0;
  static constexpr double ROT_KD = 0.4;

  // Fine alignment phase (slow, precise)
  static constexpr double FINE_ROT_KP = 3.0; // Higher P for precision
  static constexpr double FINE_ROT_KI =
      0.1; // Small I to eliminate steady-state error
  static constexpr double FINE_ROT_KD = 0.8; // Higher D to prevent overshoot
  static constexpr double FINE_THRESHOLD =
      0.0300; // Switch to fine phase at ~1.7 degrees
  static constexpr double FINE_TOLERANCE =
      0.0010; // Complete at 0.001 rad = ~0.06 degrees
  static constexpr double SETTLE_TIME =
      0.080; // 80ms settle time (timestep-independent)

  // Wall correction rate limit (rad/s) - timestep-independent
  static constexpr double WALL_CORRECT_RATE = 3.0;

  static constexpr double WALL_GAIN = 3.0;
  static constexpr double WALL_KI =
      5.0; // Increased to 5.0 to eliminate steady-state drift
  static constexpr double WALL_MAX_CORRECT = 0.8;
  static constexpr double WALL_INT_MAX = 0.3; // Anti-windup limit
};

// ============================================================================
// FAST DRIVE CONFIG (Optimized 1-tile movement)
// ============================================================================
struct FastDriveConfig {
  // Target and tolerances
  static constexpr double TARGET_DIST = 0.25000;  // 1 tile = 0.25m
  static constexpr double POSITION_TOL = 0.00300; // 3mm accuracy
  static constexpr double HEADING_TOL = 0.00500;  // ~0.3 deg accuracy
  static constexpr double SETTLE_TIME = 0.04000;  // 40ms settle

  // Optimized deceleration zones - start later, brake harder
  static constexpr double DECEL_ZONE_1 =
      0.08000; // Start decel at 8cm remaining
  static constexpr double DECEL_ZONE_2 = 0.04000; // Mid decel at 4cm
  static constexpr double DECEL_ZONE_3 = 0.01500; // Final approach at 1.5cm
  static constexpr double SPEED_ZONE_1 = 5.50;    // Speed in zone 1
  static constexpr double SPEED_ZONE_2 = 3.00;    // Speed in zone 2
  static constexpr double SPEED_ZONE_3 = 1.50;    // Speed in zone 3

  // Wall failsafe - stop if front sensors detect wall closer than this
  static constexpr double WALL_STOP_DIST = 0.09000; // 9cm = 125mm robot center

  // Wall correction
  static constexpr double WALL_APPROACH_SPEED = 1.50; // Speed for correction
  static constexpr double WALL_FINAL_TOLERANCE = 0.00200; // 2mm tolerance
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
  enum class TurnPhase { Coarse, Fine };       // Two-phase turning
  enum class DrivePhase { Moving, FineAlign }; // Two-phase driving

  // Fast drive state machine (for moveForwardFast)
  enum class FastDriveState {
    Idle,
    Accelerating,
    Cruising,
    Decelerating,
    Settling
  };

private:
  webots::Robot *robot_;
  webots::Motor *leftMotor_;
  webots::Motor *rightMotor_;
  Sensor::Sensing *sensors_;
  std::ofstream debugFile_;
  std::ofstream fastDebugFile_; // Separate debug file for fast drive

  PID distPID_;
  PID anglePID_;
  PID rotPID_;
  PID fineRotPID_; // Separate PID for fine alignment

  State state_ = State::Idle;
  DriveMode driveMode_ = DriveMode::Normal;
  TurnPhase turnPhase_ = TurnPhase::Coarse;
  DrivePhase drivePhase_ = DrivePhase::Moving;
  double settleTime_ = 0.0; // Accumulated settle time for turn settling
  double driveSettleTime_ =
      0.0; // Accumulated settle time for drive heading alignment
  double driveAlignTime_ = 0.0; // Total time spent in fine alignment phase

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

  // Wall correction smoothing - prevents jerk at wall transitions
  double prevWallCorrection_ = 0.0;
  double wallIntegral_ = 0.0; // Integral term for wall correction

  // Fast drive state variables
  FastDriveState fastDriveState_ = FastDriveState::Idle;
  double fastCurrentSpeed_ = 0.0;
  double fastTargetOdomDist_ = 0.0;
  double fastStartEncL_ = 0.0;
  double fastStartEncR_ = 0.0;

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

    // Pure odometry heading
    double dL = (encL - lastEncL_) * MazeConfig::WHEEL_RADIUS;
    double dR = (encR - lastEncR_) * MazeConfig::WHEEL_RADIUS;
    double deltaOdom = (dR - dL) / MazeConfig::AXLE_LENGTH;

    lastEncL_ = encL;
    lastEncR_ = encR;

    if (state_ == State::Rotating) {
      // During rotation: apply slip factor (wheels slip, less actual
      // rotation)
      deltaOdom *= MazeConfig::TURN_SLIP_FACTOR;
      odomHeading_ = normalizeAngle(odomHeading_ + deltaOdom);
    } else {
      // During forward movement: apply distance-based drift correction
      double distStep = (dL + dR) / 2.0;
      double driftCorrection = distStep * MazeConfig::DRIFT_PER_METER;
      odomHeading_ = normalizeAngle(odomHeading_ + deltaOdom - driftCorrection);
    }

    fusedHeading_ = odomHeading_;
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

  double getWallCorrection(double dt) {
    if (!sensors_)
      return 0.0;

    double dL = sensors_->getDistance(5);
    double dR = sensors_->getDistance(2);
    double rawCorrection = 0.0;

    if (dL < 0.15 && dR < 0.15) {
      double error = (dL - dR) / 2.0;

      // Update integral term for drift elimination
      wallIntegral_ += error * dt;
      // Anti-windup
      wallIntegral_ =
          std::max(-PIDParams::WALL_INT_MAX,
                   std::min(PIDParams::WALL_INT_MAX, wallIntegral_));

      rawCorrection =
          (error * PIDParams::WALL_GAIN) + (wallIntegral_ * PIDParams::WALL_KI);
    } else if (dL < MazeConfig::WALL_CLEARANCE) {
      // Clear integral when not in two-wall corridor
      wallIntegral_ = 0.0;
      rawCorrection = -(MazeConfig::WALL_CLEARANCE - dL) * PIDParams::WALL_GAIN;
    } else if (dR < MazeConfig::WALL_CLEARANCE) {
      // Clear integral when not in two-wall corridor
      wallIntegral_ = 0.0;
      rawCorrection = (MazeConfig::WALL_CLEARANCE - dR) * PIDParams::WALL_GAIN;
    } else {
      // Clear integral when no walls
      wallIntegral_ = 0.0;
    }

    // Clamp raw correction to limits
    rawCorrection =
        std::max(-PIDParams::WALL_MAX_CORRECT,
                 std::min(PIDParams::WALL_MAX_CORRECT, rawCorrection));

    // Transition-only smoothing: timestep-independent rate limiting
    // Limits change to WALL_CORRECT_RATE rad/s when large jumps occur
    double change = rawCorrection - prevWallCorrection_;
    double maxChange =
        PIDParams::WALL_CORRECT_RATE * dt; // Timestep-independent
    if (std::abs(change) > 0.05) {
      // Large transition detected - apply rate limit
      if (change > maxChange) {
        rawCorrection = prevWallCorrection_ + maxChange;
      } else if (change < -maxChange) {
        rawCorrection = prevWallCorrection_ - maxChange;
      }
    }

    prevWallCorrection_ = rawCorrection;
    return rawCorrection;
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

    // Deceleration zones - start earlier to compensate for higher speed
    if (remaining < 0.15) // Was 0.10 - start braking earlier
      speed = std::min(speed, 4.5);
    if (remaining < 0.08) // Was 0.05
      speed = std::min(speed, 2.5);
    if (remaining < 0.03) // Was 0.02
      speed = std::min(speed, 1.2);

    return speed;
  }

  void logDebug(double dist, double rem, double yaw, double err, double turn,
                double wall, double wallInt, double spd, double leftSpd,
                double rightSpd) {
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
    debugFile_ << "TurnCorr: " << turn << " | WallCorr: " << wall
               << " | WallInt: " << wallInt << "\n";
    debugFile_ << "Sensors(L/R/F0/F7): " << sensors_->getDistance(5) << " / "
               << sensors_->getDistance(2) << " / " << sensors_->getDistance(0)
               << " / " << sensors_->getDistance(7) << "\n";
    debugFile_ << "Speed: " << spd << " | Motors(L/R): " << leftSpd << " / "
               << rightSpd << "\n";
    debugFile_ << "----------------------------------------\n";
  }

  void logRotateDebug(double yaw, double err, double turnSpd, double leftSpd,
                      double rightSpd) {
    if (!debugFile_.is_open())
      return;

    debugFile_ << std::fixed << std::setprecision(4);
    debugFile_ << "--- TIMESTAMP: " << robot_->getTime() << " ---\n";
    debugFile_ << "State: ROTATING | Phase: "
               << (turnPhase_ == TurnPhase::Coarse ? "COARSE" : "FINE") << "\n";

    debugFile_ << std::setprecision(5);
    debugFile_ << "Yaw: " << yaw << " | Tgt: " << targetHeading_
               << " | Err: " << err << "\n";
    debugFile_ << "OdomYaw: " << odomHeading_
               << " | GyroYaw: " << sensors_->getYaw() << "\n";
    debugFile_ << "TurnSpeed: " << turnSpd << " | SettleTime: " << settleTime_
               << " / " << PIDParams::SETTLE_TIME << "s\n";
    debugFile_ << "Motors(L/R): " << leftSpd << " / " << rightSpd << "\n";
    debugFile_ << "Sensors(L/R/F0/F7): " << sensors_->getDistance(5) << " / "
               << sensors_->getDistance(2) << " / " << sensors_->getDistance(0)
               << " / " << sensors_->getDistance(7) << "\n";
    debugFile_ << "----------------------------------------\n";
  }

  // =========================================================================
  // FAST DRIVE HELPER FUNCTIONS
  // =========================================================================

  // Get distance traveled using encoders for fast drive
  double getFastOdometryDistance() {
    if (!sensors_)
      return 0.0;
    double encL = sensors_->getLeftEncoder();
    double encR = sensors_->getRightEncoder();
    double dL = (encL - fastStartEncL_) * MazeConfig::WHEEL_RADIUS;
    double dR = (encR - fastStartEncR_) * MazeConfig::WHEEL_RADIUS;
    return (dL + dR) / 2.0;
  }

  // Set motor velocities directly (renamed to avoid conflict with setMotors)
  void setMotorsDirect(double left, double right) {
    if (leftMotor_)
      leftMotor_->setVelocity(std::max(-MazeConfig::MAX_SPEED,
                                       std::min(MazeConfig::MAX_SPEED, left)));
    if (rightMotor_)
      rightMotor_->setVelocity(std::max(
          -MazeConfig::MAX_SPEED, std::min(MazeConfig::MAX_SPEED, right)));
  }

  // Compute speed based on optimized deceleration zones
  double computeFastSpeed(double remaining) {
    if (remaining > FastDriveConfig::DECEL_ZONE_1) {
      return MazeConfig::MAX_SPEED;
    } else if (remaining > FastDriveConfig::DECEL_ZONE_2) {
      return FastDriveConfig::SPEED_ZONE_1;
    } else if (remaining > FastDriveConfig::DECEL_ZONE_3) {
      return FastDriveConfig::SPEED_ZONE_2;
    } else {
      return FastDriveConfig::SPEED_ZONE_3;
    }
  }

  // Log fast drive state
  void logFastState(const char *phase, double odomDist, double remaining,
                    double speed) {
    if (!fastDebugFile_.is_open())
      return;

    // Get raw sensor values for debugging
    double rawPS0 = sensors_->getRawDistance(0); // Front-right
    double rawPS7 = sensors_->getRawDistance(7); // Front-left
    double distPS0 = sensors_->getDistance(0);
    double distPS7 = sensors_->getDistance(7);

    fastDebugFile_ << std::fixed << std::setprecision(2);
    fastDebugFile_ << std::setw(8) << robot_->getTime() << "s  ";
    fastDebugFile_ << phase << "  ";
    fastDebugFile_ << std::setprecision(5);
    fastDebugFile_ << "odom=" << std::setw(7) << odomDist;
    fastDebugFile_ << "  rem=" << std::setw(7) << remaining;
    fastDebugFile_ << "  spd=" << std::setw(5) << std::setprecision(2) << speed;
    fastDebugFile_ << "  |  PS0: raw=" << std::setw(6) << std::setprecision(0)
                   << rawPS0 << " dist=" << std::setprecision(3) << distPS0
                   << "m";
    fastDebugFile_ << "  PS7: raw=" << std::setw(6) << std::setprecision(0)
                   << rawPS7 << " dist=" << std::setprecision(3) << distPS7
                   << "m";
    fastDebugFile_ << "\n";
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
    fineRotPID_.configure(PIDParams::FINE_ROT_KP, PIDParams::FINE_ROT_KI,
                          PIDParams::FINE_ROT_KD);

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

  // Overload: move forward by 1 tile (0.25m) when called with no arguments
  void moveForward() { moveForward(1); }

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

    drivePhase_ = DrivePhase::Moving;
    driveSettleTime_ = 0.0;
    distPID_.reset();
    anglePID_.reset();
    prevWallCorrection_ = 0.0;
    wallIntegral_ = 0.0;

    std::cout << "MOVE: " << meters << "m @ heading " << targetHeading_
              << std::endl;
  }

  void turnLeft() { rotate(MazeConfig::TURN_90_RAD); }
  void turnRight() { rotate(-MazeConfig::TURN_90_RAD); }

  void rotate(double radians) {
    state_ = State::Rotating;
    turnPhase_ = TurnPhase::Coarse; // Start with fast coarse phase
    settleTime_ = 0.0;
    targetHeading_ = normalizeAngle(fusedHeading_ + radians);
    rotPID_.reset();
    fineRotPID_.reset();
    std::cout << "ROTATE: " << radians << " rad -> target " << targetHeading_
              << std::endl;
  }

  void stop() {
    state_ = State::Idle;
    setMotors(0.0, 0.0);
  }

  // =========================================================================
  // WALL DETECTION - Check if wall is detectable in front
  // Based on sensor analysis: detection starts at ~0.90m
  // =========================================================================
  bool hasWallInFront() {
    if (!sensors_)
      return false;
    sensors_->update();
    return sensors_->isWallAtFront();
  }

  // =========================================================================
  // correctToWall - Drive forward/backward to reach target distance from wall
  // Based on sensor analysis: 0.09m sensor reading = 0.125m robot center
  // =========================================================================
  void correctToWall(double targetDistM) {
    if (!robot_ || !sensors_)
      return;

    int timestep = static_cast<int>(robot_->getBasicTimeStep());
    static constexpr int MAX_ITERATIONS = 500;

    sensors_->update();
    double frontDist = sensors_->getDistanceToFrontWall();
    double error = frontDist - targetDistM;

    // Log to debug_report.txt
    if (debugFile_.is_open()) {
      debugFile_ << "\n========================================\n";
      debugFile_ << "WALL CORRECTION STARTED\n";
      debugFile_ << "Time: " << robot_->getTime() << "s\n";
      debugFile_ << "Initial distance: " << std::fixed << std::setprecision(5)
                 << frontDist << "m\n";
      debugFile_ << "Target distance: " << targetDistM << "m\n";
      debugFile_ << "Initial error: " << error << "m\n";
      debugFile_ << "Direction: "
                 << (error > 0 ? "FORWARD (too far)" : "BACKWARD (too close)")
                 << "\n";
      debugFile_ << "Raw PS0: " << sensors_->getRawDistance(0) << "\n";
      debugFile_ << "Raw PS7: " << sensors_->getRawDistance(7) << "\n";
      debugFile_ << "----------------------------------------\n";
    }

    if (fastDebugFile_.is_open()) {
      fastDebugFile_ << "\n--- WALL CORRECTION ---\n";
      fastDebugFile_ << "Initial: " << frontDist << "m, Target: " << targetDistM
                     << "m\n";
    }

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "WALL CORRECTION: " << frontDist << "m -> " << targetDistM
              << "m" << std::endl;

    int iteration = 0;
    while (std::abs(error) > FastDriveConfig::WALL_FINAL_TOLERANCE &&
           iteration < MAX_ITERATIONS) {
      if (robot_->step(timestep) == -1)
        break;

      sensors_->update();
      frontDist = sensors_->getDistanceToFrontWall();
      error = frontDist - targetDistM;

      // Bidirectional correction
      double speed = 0.0;
      if (error > FastDriveConfig::WALL_FINAL_TOLERANCE) {
        speed = FastDriveConfig::WALL_APPROACH_SPEED; // Move forward
      } else if (error < -FastDriveConfig::WALL_FINAL_TOLERANCE) {
        speed = -FastDriveConfig::WALL_APPROACH_SPEED; // Move backward
      }

      // Slow down when close
      if (std::abs(error) < 0.01000) {
        speed *= 0.5;
      }

      setMotorsDirect(speed, speed);

      if (fastDebugFile_.is_open() && iteration % 20 == 0) {
        fastDebugFile_ << "  iter=" << iteration << " dist=" << frontDist
                       << "m err=" << error << "m spd=" << speed << "\n";
      }
      iteration++;
    }

    setMotorsDirect(0.0, 0.0);

    // Settle
    for (int i = 0; i < 5 && robot_->step(timestep) != -1; i++) {
      sensors_->update();
    }

    frontDist = sensors_->getDistanceToFrontWall();
    error = frontDist - targetDistM;

    std::cout << "CORRECTION DONE: " << frontDist << "m (err=" << error
              << "m) in " << iteration << " steps" << std::endl;

    // Log correction result to debug_report.txt
    if (debugFile_.is_open()) {
      debugFile_ << "WALL CORRECTION COMPLETE\n";
      debugFile_ << "Time: " << robot_->getTime() << "s\n";
      debugFile_ << "Final distance: " << std::fixed << std::setprecision(5)
                 << frontDist << "m\n";
      debugFile_ << "Final error: " << error << "m\n";
      debugFile_ << "Iterations: " << iteration << "\n";
      debugFile_ << "Raw PS0: " << sensors_->getRawDistance(0) << "\n";
      debugFile_ << "Raw PS7: " << sensors_->getRawDistance(7) << "\n";
      debugFile_ << (std::abs(error) <= FastDriveConfig::WALL_FINAL_TOLERANCE
                         ? "SUCCESS"
                         : "TOLERANCE NOT MET")
                 << "\n";
      debugFile_ << "========================================\n\n";
      debugFile_.flush();
    }

    if (fastDebugFile_.is_open()) {
      fastDebugFile_ << "Final: " << frontDist << "m, Error: " << error
                     << "m\n";
      fastDebugFile_ << "Iterations: " << iteration << "\n";
      fastDebugFile_.flush();
    }
  }

  // =========================================================================
  // moveForwardFast - OPTIMIZED 1-TILE MOVEMENT
  // BLOCKING CALL - runs its own simulation loop until movement complete
  // Includes wall failsafe to prevent overshooting
  // =========================================================================
  void moveForwardFast() {
    if (!robot_ || !sensors_)
      return;

    int timestep = static_cast<int>(robot_->getBasicTimeStep());
    double dt = timestep / 1000.0;

    // Initialize encoder start positions
    sensors_->update();
    fastStartEncL_ = sensors_->getLeftEncoder();
    fastStartEncR_ = sensors_->getRightEncoder();
    fastTargetOdomDist_ = FastDriveConfig::TARGET_DIST;
    fastCurrentSpeed_ = 0.0;
    fastDriveState_ = FastDriveState::Accelerating;

    // Open fast debug file if not open
    if (!fastDebugFile_.is_open()) {
      fastDebugFile_.open("testing_debugs.txt",
                          std::ios::out | std::ios::trunc);
      if (fastDebugFile_.is_open()) {
        fastDebugFile_ << "=== FAST DRIVE DEBUG LOG ===\n";
        fastDebugFile_ << "Fast drive target: " << FastDriveConfig::TARGET_DIST
                       << "m\n";
        fastDebugFile_ << "Decel zones: " << FastDriveConfig::DECEL_ZONE_1
                       << "m, " << FastDriveConfig::DECEL_ZONE_2 << "m, "
                       << FastDriveConfig::DECEL_ZONE_3 << "m\n";
        fastDebugFile_ << "====================================\n\n";
      }
    }

    if (fastDebugFile_.is_open()) {
      fastDebugFile_ << "\n--- NEW FAST RUN ---\n";
      fastDebugFile_ << "Target: " << fastTargetOdomDist_ << "m\n\n";
    }

    // Run internal update loop until movement is complete
    bool running = true;
    while (running && robot_->step(timestep) != -1) {
      sensors_->update();

      double odomDist = getFastOdometryDistance();
      double remaining = fastTargetOdomDist_ - odomDist;

      // WALL FAILSAFE: Stop if front sensors detect wall too close
      double frontDist = sensors_->getDistanceToFrontWall();
      if (frontDist < FastDriveConfig::WALL_STOP_DIST && frontDist < 1.5) {
        setMotorsDirect(0.0, 0.0);

        std::cout << std::fixed << std::setprecision(5);
        std::cout << "WALL FAILSAFE: Stopping at " << frontDist << "m (< "
                  << FastDriveConfig::WALL_STOP_DIST << "m)" << std::endl;

        // Log wall failsafe to debug_report.txt
        if (debugFile_.is_open()) {
          debugFile_ << "\n========================================\n";
          debugFile_ << "*** WALL FAILSAFE TRIGGERED ***\n";
          debugFile_ << "Time: " << robot_->getTime() << "s\n";
          debugFile_ << "Front distance: " << std::fixed << std::setprecision(5)
                     << frontDist << "m\n";
          debugFile_ << "Stop threshold: " << FastDriveConfig::WALL_STOP_DIST
                     << "m\n";
          debugFile_ << "Odometry traveled: " << odomDist << "m\n";
          debugFile_ << "Remaining to target: " << remaining << "m\n";
          debugFile_ << "Current speed: " << fastCurrentSpeed_ << " rad/s\n";
          debugFile_ << "Raw PS0: " << sensors_->getRawDistance(0) << "\n";
          debugFile_ << "Raw PS7: " << sensors_->getRawDistance(7) << "\n";
          debugFile_ << "PS0 dist: " << sensors_->getDistance(0) << "m\n";
          debugFile_ << "PS7 dist: " << sensors_->getDistance(7) << "m\n";
          debugFile_ << "========================================\n\n";
          debugFile_.flush();
        }

        if (fastDebugFile_.is_open()) {
          fastDebugFile_ << "\n*** WALL FAILSAFE TRIGGERED ***\n";
          fastDebugFile_ << "Front dist: " << frontDist << "m\n";
          fastDebugFile_ << "Stop threshold: "
                         << FastDriveConfig::WALL_STOP_DIST << "m\n";
          fastDebugFile_ << "Odometry: " << odomDist << "m\n";
          fastDebugFile_.flush();
        }

        running = false;
        break;
      }

      // Compute target speed based on remaining distance
      double targetSpeed = computeFastSpeed(remaining);

      // Smooth speed transitions
      if (remaining > FastDriveConfig::DECEL_ZONE_1) {
        fastCurrentSpeed_ =
            std::min(fastCurrentSpeed_ + 15.0 * dt, targetSpeed);
      } else {
        if (fastCurrentSpeed_ > targetSpeed) {
          fastCurrentSpeed_ =
              std::max(fastCurrentSpeed_ - 25.0 * dt, targetSpeed);
        } else {
          fastCurrentSpeed_ = targetSpeed;
        }
      }

      setMotorsDirect(fastCurrentSpeed_, fastCurrentSpeed_);
      logFastState("RUN   ", odomDist, remaining, fastCurrentSpeed_);

      // Check if reached target
      if (remaining <= FastDriveConfig::POSITION_TOL) {
        setMotorsDirect(0.0, 0.0);

        // Brief settling period
        for (int i = 0; i < 4 && robot_->step(timestep) != -1; i++) {
          sensors_->update();
        }

        double finalDist = getFastOdometryDistance();
        double error = finalDist - FastDriveConfig::TARGET_DIST;

        std::cout << std::fixed << std::setprecision(5);
        std::cout << "FAST COMPLETE: dist=" << finalDist << "m (error=" << error
                  << "m)" << std::endl;

        if (fastDebugFile_.is_open()) {
          fastDebugFile_ << "\n========== FAST RUN COMPLETE ==========\n";
          fastDebugFile_ << std::fixed << std::setprecision(5);
          fastDebugFile_ << "Final distance: " << finalDist << "m\n";
          fastDebugFile_ << "Target was: " << FastDriveConfig::TARGET_DIST
                         << "m\n";
          fastDebugFile_ << "Error: " << error << "m\n";
          fastDebugFile_ << "========================================\n";
          fastDebugFile_.flush();
        }

        running = false;
      }
    }

    fastDriveState_ = FastDriveState::Idle;
  }

  // =========================================================================
  // STATUS QUERIES
  // =========================================================================
  bool isBusy() const { return state_ != State::Idle; }
  State getState() const { return state_; }
  bool isFastDriveActive() const {
    return fastDriveState_ != FastDriveState::Idle;
  }

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
      double headingError = normalizeAngle(targetHeading_ - fusedHeading_);

      if (drivePhase_ == DrivePhase::Moving) {
        // Normal driving phase
        double baseSpeed = computeBaseSpeed(remaining);
        double distOutput = distPID_.compute(remaining, dt);
        double speedCmd = std::min(baseSpeed, std::abs(distOutput));
        if (distOutput < 0)
          speedCmd = -speedCmd;

        double turnCorrection = anglePID_.compute(headingError, dt);
        turnCorrection =
            std::max(-PIDParams::ANGLE_MAX_CORRECT,
                     std::min(PIDParams::ANGLE_MAX_CORRECT, turnCorrection));

        double wallCorrection = 0.0;
        if (remaining > 0.03) {
          wallCorrection = getWallCorrection(dt);
        }

        double totalTurn = turnCorrection + wallCorrection;
        double leftSpeed = speedCmd - totalTurn;
        double rightSpeed = speedCmd + totalTurn;
        setMotors(leftSpeed, rightSpeed);

        logDebug(currentDist, remaining, fusedHeading_, headingError,
                 turnCorrection, wallCorrection, wallIntegral_, speedCmd,
                 leftSpeed, rightSpeed);

        // Check if distance target reached - switch to fine alignment
        if (std::abs(remaining) < 0.005) {
          setMotors(0.0, 0.0);
          drivePhase_ = DrivePhase::FineAlign;
          driveSettleTime_ = 0.0;
          driveAlignTime_ = 0.0; // Reset alignment timer
          fineRotPID_.reset();
          std::cout << "DRIVE: Switching to FINE heading alignment (err="
                    << headingError << ")" << std::endl;
        }
      } else {
        // Fine alignment phase - correct heading drift at end
        driveAlignTime_ += dt; // Track total alignment time
        double turnSpeed = fineRotPID_.compute(headingError, dt);
        turnSpeed = std::max(-0.5, std::min(0.5, turnSpeed));
        setMotors(-turnSpeed, turnSpeed);

        // Check for timeout - prevents infinite alignment loops at low
        // timesteps
        if (driveAlignTime_ >= PIDParams::DRIVE_MAX_ALIGN_TIME) {
          stop();
          std::cout << "ARRIVED: dist=" << currentDist << "m yaw=" << std::fixed
                    << std::setprecision(5) << fusedHeading_
                    << " (timeout, err=" << headingError << ")" << std::endl;
        } else if (std::abs(headingError) < PIDParams::DRIVE_FINE_TOLERANCE) {
          driveSettleTime_ += dt; // Accumulate settle time
          if (driveSettleTime_ >= PIDParams::DRIVE_SETTLE_TIME) {
            stop();
            std::cout << "ARRIVED: dist=" << currentDist
                      << "m yaw=" << std::fixed << std::setprecision(5)
                      << fusedHeading_ << " (aligned)" << std::endl;
          }
        } else {
          driveSettleTime_ = 0.0; // Reset if out of tolerance
        }
      }
    } else if (state_ == State::Rotating) {
      double headingError = normalizeAngle(targetHeading_ - fusedHeading_);
      double turnSpeed = 0.0;

      if (turnPhase_ == TurnPhase::Coarse) {
        // Coarse phase: fast rotation until close to target
        turnSpeed = rotPID_.compute(headingError, dt);
        turnSpeed = std::max(-3.0, std::min(3.0, turnSpeed));

        // Switch to fine phase when close enough
        if (std::abs(headingError) < PIDParams::FINE_THRESHOLD) {
          turnPhase_ = TurnPhase::Fine;
          fineRotPID_.reset();
          settleTime_ = 0.0;
          std::cout << "TURN: Switching to FINE alignment (err=" << headingError
                    << ")" << std::endl;
        }
      } else {
        // Fine phase: slow, precise alignment
        turnSpeed = fineRotPID_.compute(headingError, dt);
        turnSpeed = std::max(-0.5, std::min(0.5, turnSpeed));

        // Check if within final tolerance
        if (std::abs(headingError) < PIDParams::FINE_TOLERANCE) {
          settleTime_ += dt; // Accumulate settle time
          if (settleTime_ >= PIDParams::SETTLE_TIME) {
            stop();
            std::cout << "TURN COMPLETE: yaw=" << std::fixed
                      << std::setprecision(5) << fusedHeading_
                      << " target=" << targetHeading_ << " err=" << headingError
                      << std::endl;
            return;
          }
        } else {
          settleTime_ = 0.0; // Reset if out of tolerance
        }
      }

      setMotors(-turnSpeed, turnSpeed);
      logRotateDebug(fusedHeading_, headingError, turnSpeed, -turnSpeed,
                     turnSpeed);
    }
  }
};

} // namespace Motion

#endif // MOTION_CONTROL_H
