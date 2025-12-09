#ifndef TESTING_FUNCTIONS_H
#define TESTING_FUNCTIONS_H

#include "motion_control.h"
#include "sensing.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

namespace Testing {

class TestingFunctions {
private:
  webots::Robot *robot_;
  Sensor::Sensing *sensors_;
  Motion::MovingController *motion_;

  // Motors for direct control
  webots::Motor *leftMotor_;
  webots::Motor *rightMotor_;

  // Debug file
  std::ofstream debugFile_;

  // =========================================================================
  // CONSTANTS
  // =========================================================================
  static constexpr double TILE_SIZE = 0.25;
  static constexpr double TARGET_STOP_DIST = 0.125;     // Stop 12.5cm from wall
  static constexpr double WALL_CLOSE_THRESHOLD = 0.25;  // ShortRun if closer
  static constexpr double TRANSITION_THRESHOLD = 0.40;  // LongRun→ShortRun
  static constexpr double FINE_TUNE_THRESHOLD = 0.15;   // Switch to sensors
  static constexpr double POSITION_TOLERANCE = 0.00500; // 5mm accuracy
  static constexpr double SETTLE_TIME = 0.08000;        // 80ms settle
  static constexpr double MAX_SPEED = 6.28;
  static constexpr double SHORT_RUN_SPEED = 3.0;
  static constexpr double RAMP_RATE = 8.0; // rad/s² accel/decel
  static constexpr double WHEEL_RADIUS = 0.02001;

  // =========================================================================
  // FAST SINGLE-TILE CONSTANTS (Strategy 2: Optimized Deceleration Zones)
  // =========================================================================
  static constexpr double FAST_TARGET_DIST = 0.25000;  // 1 tile = 0.25m
  static constexpr double FAST_POSITION_TOL = 0.00300; // 3mm accuracy (was 5mm)
  static constexpr double FAST_HEADING_TOL = 0.00500;  // ~0.3 deg accuracy
  static constexpr double FAST_SETTLE_TIME = 0.04000;  // 40ms settle (reduced)

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

  // =========================================================================
  // STATE MACHINE
  // =========================================================================
  enum class WallDriveState {
    Idle,
    ShortRun,      // Wall close - moderate speed PID approach
    LongRunAccel,  // Ramping to max speed
    LongRunCruise, // At max speed, waiting for wall
    LongRunDecel,  // Decelerating to transition
    FineSettle     // Sensor-based final positioning
  };

  WallDriveState wallDriveState_ = WallDriveState::Idle;

  // State machine for moveForwardFast (1-tile optimized)
  enum class FastDriveState {
    Idle,
    Accelerating, // Ramp up to max speed
    Cruising,     // At max speed
    Decelerating, // Zone-based deceleration
    Settling      // Final position settling
  };

  FastDriveState fastDriveState_ = FastDriveState::Idle;

  // =========================================================================
  // PID CONTROLLER (copied from Motion namespace pattern)
  // =========================================================================
  class SimplePID {
  private:
    double kp_, ki_, kd_;
    double integral_ = 0.0;
    double prevError_ = 0.0;
    double maxOutput_ = 6.28;

  public:
    void configure(double kp, double ki, double kd, double maxOut = 6.28) {
      kp_ = kp;
      ki_ = ki;
      kd_ = kd;
      maxOutput_ = maxOut;
    }

    void reset() {
      integral_ = 0.0;
      prevError_ = 0.0;
    }

    double compute(double error, double dt) {
      if (dt <= 0.0001)
        return 0.0;

      double pOut = kp_ * error;

      integral_ += error * dt;
      integral_ = std::max(-3.0, std::min(3.0, integral_)); // Anti-windup
      double iOut = ki_ * integral_;

      double derivative = (error - prevError_) / dt;
      prevError_ = error;
      double dOut = kd_ * derivative;

      double output = pOut + iOut + dOut;
      return std::max(-maxOutput_, std::min(maxOutput_, output));
    }
  };

  SimplePID distPID_;

  // =========================================================================
  // STATE VARIABLES
  // =========================================================================
  double currentSpeed_ = 0.0;
  double targetOdomDist_ = 0.0; // Target distance from start (odometry)
  double startEncL_ = 0.0;
  double startEncR_ = 0.0;
  double settleTime_ = 0.0;
  // ouble initialFrontDist_ = 0.0; // Front distance at start

  // =========================================================================
  // HELPER FUNCTIONS
  // =========================================================================

  double getOdometryDistance() {
    if (!sensors_)
      return 0.0;
    double encL = sensors_->getLeftEncoder();
    double encR = sensors_->getRightEncoder();
    double dL = (encL - startEncL_) * WHEEL_RADIUS;
    double dR = (encR - startEncR_) * WHEEL_RADIUS;
    return (dL + dR) / 2.0;
  }

  void setMotorsDirect(double left, double right) {
    if (leftMotor_)
      leftMotor_->setVelocity(std::max(-MAX_SPEED, std::min(MAX_SPEED, left)));
    if (rightMotor_)
      rightMotor_->setVelocity(
          std::max(-MAX_SPEED, std::min(MAX_SPEED, right)));
  }

  // Compute speed based on optimized deceleration zones (Strategy 2)
  double computeFastSpeed(double remaining) {
    if (remaining < DECEL_ZONE_3)
      return SPEED_ZONE_3; // Final approach: 1.5 rad/s
    if (remaining < DECEL_ZONE_2)
      return SPEED_ZONE_2; // Mid decel: 3.0 rad/s
    if (remaining < DECEL_ZONE_1)
      return SPEED_ZONE_1; // Start decel: 5.5 rad/s
    return MAX_SPEED;      // Full speed: 6.28 rad/s
  }

  void logFastState(const char *phase, double odomDist, double remaining,
                    double speed) {
    if (!debugFile_.is_open())
      return;

    // Get raw sensor values for debugging
    double rawPS0 = sensors_->getRawDistance(0); // Front-right
    double rawPS7 = sensors_->getRawDistance(7); // Front-left
    double distPS0 = sensors_->getDistance(0);
    double distPS7 = sensors_->getDistance(7);

    debugFile_ << std::fixed << std::setprecision(2);
    debugFile_ << std::setw(8) << robot_->getTime() << "s  ";
    debugFile_ << phase << "  ";
    debugFile_ << std::setprecision(5);
    debugFile_ << "odom=" << std::setw(7) << odomDist;
    debugFile_ << "  rem=" << std::setw(7) << remaining;
    debugFile_ << "  spd=" << std::setw(5) << std::setprecision(2) << speed;
    debugFile_ << "  |  PS0: raw=" << std::setw(6) << std::setprecision(0)
               << rawPS0 << " dist=" << std::setprecision(3) << distPS0 << "m";
    debugFile_ << "  PS7: raw=" << std::setw(6) << std::setprecision(0)
               << rawPS7 << " dist=" << std::setprecision(3) << distPS7 << "m";
    debugFile_ << "\n";
  }

public:
  // =========================================================================
  // CONSTRUCTOR / DESTRUCTOR
  // =========================================================================
  TestingFunctions(webots::Robot *r, Sensor::Sensing *s,
                   Motion::MovingController *m)
      : robot_(r), sensors_(s), motion_(m) {
    leftMotor_ = robot_->getMotor("left wheel motor");
    rightMotor_ = robot_->getMotor("right wheel motor");

    if (leftMotor_)
      leftMotor_->setPosition(INFINITY);
    if (rightMotor_)
      rightMotor_->setPosition(INFINITY);

    debugFile_.open("testing_debugs.txt", std::ios::out | std::ios::trunc);
    if (debugFile_.is_open()) {
      debugFile_ << "=== TESTING FUNCTIONS DEBUG LOG ===\n";
      debugFile_ << "Fast drive target: " << FAST_TARGET_DIST << "m\n";
      debugFile_ << "Decel zones: " << DECEL_ZONE_1 << "m, " << DECEL_ZONE_2
                 << "m, " << DECEL_ZONE_3 << "m\n";
      debugFile_ << "====================================\n\n";
    }
  }

  ~TestingFunctions() {
    if (debugFile_.is_open())
      debugFile_.close();
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
  // correctToWall - Drive forward until reaching target distance from wall
  // Based on sensor analysis: 0.09m sensor reading = 0.125m robot center to
  // wall
  // =========================================================================
  void correctToWall(double targetDistM) {
    if (!robot_ || !sensors_)
      return;

    int timestep = static_cast<int>(robot_->getBasicTimeStep());

    // Constants
    static constexpr double APPROACH_SPEED = 1.50;     // rad/s - slow approach
    static constexpr double FINAL_TOLERANCE = 0.00200; // 2mm tolerance
    static constexpr double MAX_ITERATIONS = 500;      // ~8 seconds safety

    sensors_->update();
    double frontDist = sensors_->getDistanceToFrontWall();
    double error = frontDist - targetDistM;

    if (debugFile_.is_open()) {
      debugFile_ << "\n--- WALL CORRECTION ---\n";
      debugFile_ << "Initial: " << frontDist << "m, Target: " << targetDistM
                 << "m\n";
    }

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "WALL CORRECTION: " << frontDist << "m -> " << targetDistM
              << "m" << std::endl;

    int iteration = 0;
    // Use absolute error for loop condition - correct in BOTH directions
    while (std::abs(error) > FINAL_TOLERANCE && iteration < MAX_ITERATIONS) {
      if (robot_->step(timestep) == -1)
        break;

      sensors_->update();
      frontDist = sensors_->getDistanceToFrontWall();
      error = frontDist - targetDistM;

      // Bidirectional correction:
      // error > 0: too far from wall, move FORWARD
      // error < 0: too close to wall, move BACKWARD
      double speed = 0.0;
      if (error > FINAL_TOLERANCE) {
        speed = APPROACH_SPEED; // Move forward
      } else if (error < -FINAL_TOLERANCE) {
        speed = -APPROACH_SPEED; // Move backward
      }

      // Slow down when close to target
      if (std::abs(error) < 0.01000) { // Within 1cm
        speed *= 0.5;
      }

      setMotorsDirect(speed, speed);

      if (debugFile_.is_open() && iteration % 20 == 0) {
        debugFile_ << "  iter=" << iteration << " dist=" << frontDist
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

    if (debugFile_.is_open()) {
      debugFile_ << "Final: " << frontDist << "m, Error: " << error << "m\n";
      debugFile_ << "Iterations: " << iteration << "\n";
      debugFile_.flush();
    }
  }

  // =========================================================================
  // moveForwardFast - OPTIMIZED 1-TILE MOVEMENT (Strategy 2)
  // BLOCKING CALL - runs its own simulation loop until movement complete
  // =========================================================================
  void moveForwardFast() {
    if (!robot_ || !sensors_)
      return;

    // Get timestep from robot
    int timestep = static_cast<int>(robot_->getBasicTimeStep());
    double dt = timestep / 1000.0;

    // Record start encoder values
    startEncL_ = sensors_->getLeftEncoder();
    startEncR_ = sensors_->getRightEncoder();
    targetOdomDist_ = FAST_TARGET_DIST;
    currentSpeed_ = 0.0;
    settleTime_ = 0.0;

    std::cout << "--- moveForwardFast START ---" << std::endl;
    std::cout << "Target distance: " << targetOdomDist_ << "m" << std::endl;

    if (debugFile_.is_open()) {
      debugFile_ << "\n--- NEW FAST RUN ---\n";
      debugFile_ << "Target: " << targetOdomDist_ << "m\n\n";
    }

    // Run internal update loop until movement is complete
    bool running = true;
    bool wallFailsafe = false;
    while (running && robot_->step(timestep) != -1) {
      // CRITICAL: Update sensors to get fresh encoder readings
      sensors_->update();

      double odomDist = getOdometryDistance();
      double remaining = targetOdomDist_ - odomDist;

      // =====================================================================
      // WALL FAILSAFE: Stop if front sensors detect wall too close
      // This prevents overshooting when approaching a wall
      // =====================================================================
      double frontDist = sensors_->getDistanceToFrontWall();
      if (frontDist < WALL_STOP_DIST &&
          frontDist < 1.5) { // < 9cm and valid reading
        // Wall detected too close - stop immediately!
        setMotorsDirect(0.0, 0.0);
        wallFailsafe = true;

        std::cout << std::fixed << std::setprecision(5);
        std::cout << "WALL FAILSAFE: Stopping at " << frontDist << "m (< "
                  << WALL_STOP_DIST << "m)" << std::endl;

        if (debugFile_.is_open()) {
          debugFile_ << "\n*** WALL FAILSAFE TRIGGERED ***\n";
          debugFile_ << "Front dist: " << frontDist << "m\n";
          debugFile_ << "Stop threshold: " << WALL_STOP_DIST << "m\n";
          debugFile_ << "Odometry: " << odomDist << "m\n";
          debugFile_.flush();
        }

        running = false;
        break;
      }

      // Compute target speed based on remaining distance
      double targetSpeed = computeFastSpeed(remaining);

      // Smooth speed transitions
      if (remaining > DECEL_ZONE_1) {
        // Accelerating phase - ramp up
        currentSpeed_ = std::min(currentSpeed_ + 15.0 * dt, targetSpeed);
      } else {
        // Decelerating phase - ramp down
        if (currentSpeed_ > targetSpeed) {
          currentSpeed_ = std::max(currentSpeed_ - 25.0 * dt, targetSpeed);
        } else {
          currentSpeed_ = targetSpeed;
        }
      }

      setMotorsDirect(currentSpeed_, currentSpeed_);
      logFastState("RUN   ", odomDist, remaining, currentSpeed_);

      // Check if reached target (only if wall failsafe didn't trigger)
      if (remaining <= FAST_POSITION_TOL) {
        // Stop motors and settle
        setMotorsDirect(0.0, 0.0);

        // Brief settling period
        for (int i = 0; i < 4 && robot_->step(timestep) != -1; i++) {
          sensors_->update();
        }

        double finalDist = getOdometryDistance();
        double error = finalDist - FAST_TARGET_DIST;

        std::cout << std::fixed << std::setprecision(5);
        std::cout << "FAST COMPLETE: dist=" << finalDist << "m (error=" << error
                  << "m)" << std::endl;

        if (debugFile_.is_open()) {
          debugFile_ << "\n========== FAST RUN COMPLETE ==========\n";
          debugFile_ << std::fixed << std::setprecision(5);
          debugFile_ << "Final distance: " << finalDist << "m\n";
          debugFile_ << "Target was: " << FAST_TARGET_DIST << "m\n";
          debugFile_ << "Error: " << error << "m\n";
          debugFile_ << "========================================\n";
          debugFile_.flush();
        }

        running = false;
      }
    }
  }

  // =========================================================================
  // updateFast(dt) - CALL EVERY TIMESTEP FOR moveForwardFast
  // =========================================================================
  void updateFast(double dt) {
    if (fastDriveState_ == FastDriveState::Idle || !sensors_)
      return;

    double odomDist = getOdometryDistance();
    double remaining = targetOdomDist_ - odomDist;

    switch (fastDriveState_) {

    // -----------------------------------------------------------------------
    // ACCELERATING - Ramp up to max speed
    // -----------------------------------------------------------------------
    case FastDriveState::Accelerating: {
      double targetSpeed = computeFastSpeed(remaining);
      // Ramp up smoothly
      currentSpeed_ = std::min(currentSpeed_ + 15.0 * dt, targetSpeed);
      setMotorsDirect(currentSpeed_, currentSpeed_);

      // Check if we need to start decelerating
      if (remaining < DECEL_ZONE_1) {
        fastDriveState_ = FastDriveState::Decelerating;
        std::cout << "Fast: Entering DECEL at " << odomDist
                  << "m (rem=" << remaining << "m)" << std::endl;
      }

      logFastState("ACCEL ", odomDist, remaining, currentSpeed_);
      break;
    }

    // -----------------------------------------------------------------------
    // DECELERATING - Zone-based speed reduction
    // -----------------------------------------------------------------------
    case FastDriveState::Decelerating: {
      double targetSpeed = computeFastSpeed(remaining);

      // Smooth speed transition (don't jump, ramp down)
      if (currentSpeed_ > targetSpeed) {
        currentSpeed_ = std::max(currentSpeed_ - 20.0 * dt, targetSpeed);
      } else {
        currentSpeed_ = targetSpeed;
      }

      setMotorsDirect(currentSpeed_, currentSpeed_);

      // Check if reached target
      if (remaining <= FAST_POSITION_TOL) {
        fastDriveState_ = FastDriveState::Settling;
        setMotorsDirect(0.0, 0.0);
        settleTime_ = 0.0;
        std::cout << "Fast: Entering SETTLE at " << odomDist << "m"
                  << std::endl;
      }

      logFastState("DECEL ", odomDist, remaining, currentSpeed_);
      break;
    }

    // -----------------------------------------------------------------------
    // SETTLING - Wait for robot to stabilize
    // -----------------------------------------------------------------------
    case FastDriveState::Settling: {
      setMotorsDirect(0.0, 0.0);
      settleTime_ += dt;

      if (settleTime_ >= FAST_SETTLE_TIME) {
        fastDriveState_ = FastDriveState::Idle;

        double finalDist = getOdometryDistance();
        double error = finalDist - FAST_TARGET_DIST;

        std::cout << std::fixed << std::setprecision(5);
        std::cout << "FAST COMPLETE: dist=" << finalDist << "m (error=" << error
                  << "m)" << std::endl;

        if (debugFile_.is_open()) {
          debugFile_ << "\n========== FAST RUN COMPLETE ==========\n";
          debugFile_ << "Final distance: " << finalDist << "m\n";
          debugFile_ << "Target was: " << FAST_TARGET_DIST << "m\n";
          debugFile_ << "Error: " << error << "m\n";
          debugFile_ << "========================================\n";
          debugFile_.flush();
        }
      }

      logFastState("SETTLE", odomDist, remaining, 0.0);
      break;
    }

    case FastDriveState::Idle:
      break;

    case FastDriveState::Cruising:
      // Not used for 1-tile moves (too short to reach cruise)
      // Falls through to deceleration if somehow entered
      fastDriveState_ = FastDriveState::Decelerating;
      break;
    }
  }

  // =========================================================================
  // STATUS QUERIES FOR FAST DRIVE
  // =========================================================================
  bool isFastDriveActive() const {
    return fastDriveState_ != FastDriveState::Idle;
  }

  void stopFastDrive() {
    setMotorsDirect(0.0, 0.0);
    fastDriveState_ = FastDriveState::Idle;
    std::cout << "Fast drive manually stopped." << std::endl;
  }

  // =========================================================================
  // moveForwardUntilWall - ORIGINAL ENTRY POINT (commented out)
  // =========================================================================
  // void moveForwardUntilWall() {
  //   if (!robot_ || !sensors_)
  //     return;

  //   // Record start encoder values
  //   startEncL_ = sensors_->getLeftEncoder();
  //   startEncR_ = sensors_->getRightEncoder();
  //   initialFrontDist_ = getFrontDistance();
  //   currentSpeed_ = 0.0;
  //   settleTime_ = 0.0;
  //   distPID_.reset();

  //   std::cout << "--- moveForwardUntilWall START ---" << std::endl;
  //   std::cout << "Front distance: " << initialFrontDist_ << "m" << std::endl;

  //   if (isWallClose()) {
  //     // Type 1: ShortRun - wall is already close
  //     wallDriveState_ = WallDriveState::ShortRun;
  //     targetOdomDist_ = initialFrontDist_ - TARGET_STOP_DIST;
  //     currentSpeed_ = SHORT_RUN_SPEED;
  //     distPID_.configure(10.0, 0.0, 2.5, SHORT_RUN_SPEED);
  //     std::cout << "Mode: ShortRun (wall close)" << std::endl;
  //     std::cout << "Target odom distance: " << targetOdomDist_ << "m"
  //               << std::endl;
  //   } else {
  //     // Type 2: LongRun - wall is far, accelerate first
  //     wallDriveState_ = WallDriveState::LongRunAccel;
  //     targetOdomDist_ = 999.0; // Will be calculated when wall detected
  //     distPID_.configure(8.0, 0.0, 2.0, MAX_SPEED);
  //     std::cout << "Mode: LongRunAccel (wall far)" << std::endl;
  //   }

  //   if (debugFile_.is_open()) {
  //     debugFile_ << "\n--- NEW RUN ---\n";
  //     debugFile_ << "Initial front dist: " << initialFrontDist_ << "m\n";
  //     debugFile_ << "State: " << (isWallClose() ? "ShortRun" :
  //     "LongRunAccel")
  //                << "\n\n";
  //   }
  // }

  // =========================================================================
  // update(dt) - CALL EVERY TIMESTEP
  // =========================================================================
  // void update(double dt) {
  //   if (wallDriveState_ == WallDriveState::Idle || !sensors_)
  //     return;

  //   double odomDist = getOdometryDistance();
  //   double frontDist = getFrontDistance();
  //   double remaining = 0.0;

  //   switch (wallDriveState_) {

  //   //
  //   -----------------------------------------------------------------------
  // LONG RUN ACCELERATION
  //   //
  //   -----------------------------------------------------------------------
  //   case WallDriveState::LongRunAccel:
  //     currentSpeed_ = std::min(currentSpeed_ + RAMP_RATE * dt, MAX_SPEED);
  //     setMotorsDirect(currentSpeed_, currentSpeed_);

  //     if (currentSpeed_ >= MAX_SPEED * 0.95) {
  //       wallDriveState_ = WallDriveState::LongRunCruise;
  //       std::cout << "LongRun: Reached cruise speed" << std::endl;
  //     }

  //     // Check for wall detection during accel
  //     if (frontDist < TRANSITION_THRESHOLD) {
  //       wallDriveState_ = WallDriveState::LongRunDecel;
  //       std::cout << "LongRun: Wall detected at " << frontDist
  //                 << "m, decelerating" << std::endl;
  //     }

  //     logState("ACCEL ", odomDist, frontDist, -1.0, currentSpeed_);
  //     break;

  //   //
  //   -----------------------------------------------------------------------
  //   // LONG RUN CRUISE
  //   //
  //   -----------------------------------------------------------------------
  //   case WallDriveState::LongRunCruise:
  //     setMotorsDirect(MAX_SPEED, MAX_SPEED);

  //     if (frontDist < TRANSITION_THRESHOLD) {
  //       wallDriveState_ = WallDriveState::LongRunDecel;
  //       std::cout << "LongRun: Wall at " << frontDist << "m, decelerating"
  //                 << std::endl;
  //     }

  //     logState("CRUISE", odomDist, frontDist, -1.0, MAX_SPEED);
  //     break;

  //   //
  //   -----------------------------------------------------------------------
  //   // LONG RUN DECELERATION (transition to ShortRun)
  //   //
  //   -----------------------------------------------------------------------
  //   case WallDriveState::LongRunDecel:
  //     // Smooth deceleration
  //     currentSpeed_ = std::max(currentSpeed_ - RAMP_RATE * dt,
  //     SHORT_RUN_SPEED); setMotorsDirect(currentSpeed_, currentSpeed_);

  //     if (currentSpeed_ <= SHORT_RUN_SPEED) {
  //       // Transition to ShortRun
  //       wallDriveState_ = WallDriveState::ShortRun;
  //       targetOdomDist_ = odomDist + (frontDist - TARGET_STOP_DIST);
  //       distPID_.configure(10.0, 0.0, 2.5, SHORT_RUN_SPEED);
  //       distPID_.reset();
  //       std::cout << "Transition to ShortRun, target odom: " <<
  //       targetOdomDist_
  //                 << "m" << std::endl;
  //     }

  //     logState("DECEL ", odomDist, frontDist, -1.0, currentSpeed_);
  //     break;

  //   //
  //   -----------------------------------------------------------------------
  //   // SHORT RUN (PID controlled approach)
  //   //
  //   -----------------------------------------------------------------------
  //   case WallDriveState::ShortRun:
  //     remaining = targetOdomDist_ - odomDist;

  //     if (remaining > FINE_TUNE_THRESHOLD) {
  //       // Use odometry-based PID for main approach
  //       double pidOutput = distPID_.compute(remaining, dt);
  //       currentSpeed_ = std::max(0.5, std::min(SHORT_RUN_SPEED, pidOutput));
  //       setMotorsDirect(currentSpeed_, currentSpeed_);
  //     } else {
  //       // Switch to FineSettle - now trust sensors
  //       wallDriveState_ = WallDriveState::FineSettle;
  //       distPID_.configure(15.0, 0.1, 3.0, 1.2);
  //       distPID_.reset();
  //       settleTime_ = 0.0;
  //       std::cout << "ShortRun: Entering FineSettle (front=" << frontDist
  //                 << "m)" << std::endl;
  //     }

  //     logState("SHORT ", odomDist, frontDist, remaining, currentSpeed_);
  //     break;

  //   //
  //   -----------------------------------------------------------------------
  //   // FINE SETTLE (sensor-based final positioning)
  //   //
  //   -----------------------------------------------------------------------
  //   case WallDriveState::FineSettle: {
  //     // NOW trust sensor for final distance
  //     double error = frontDist - TARGET_STOP_DIST;
  //     remaining = error;

  //     if (std::abs(error) < POSITION_TOLERANCE) {
  //       // Within tolerance - accumulate settle time
  //       settleTime_ += dt;
  //       setMotorsDirect(0.0, 0.0);

  //       if (settleTime_ >= SETTLE_TIME) {
  //         // DONE!
  //         wallDriveState_ = WallDriveState::Idle;
  //         setMotorsDirect(0.0, 0.0);

  //         std::cout << std::fixed << std::setprecision(5);
  //         std::cout << "SETTLED at " << frontDist
  //                   << "m from wall (error=" << error << "m)" << std::endl;

  //         if (debugFile_.is_open()) {
  //           debugFile_ << "\n========== FINAL POSITION ==========\n";
  //           debugFile_ << "Front distance: " << frontDist << "m\n";
  //           debugFile_ << "Target was: " << TARGET_STOP_DIST << "m\n";
  //           debugFile_ << "Error: " << error << "m\n";
  //           debugFile_ << "Odom distance: " << odomDist << "m\n";
  //           debugFile_ << "=====================================\n";
  //         }

  //         // Automatically write full debug report
  //         writeDebugReport();
  //         return;
  //       }
  //     } else {
  //       // Outside tolerance - keep correcting
  //       settleTime_ = 0.0;
  //       double pidOutput = distPID_.compute(error, dt);
  //       currentSpeed_ = std::max(-1.2, std::min(1.2, pidOutput));
  //       setMotorsDirect(currentSpeed_, currentSpeed_);
  //     }

  //     logState("FINE  ", odomDist, frontDist, remaining, currentSpeed_);
  //     break;
  //   }

  //   case WallDriveState::Idle:
  //     break;
  //   }
  // }

  // //
  // =========================================================================
  // // STATUS QUERIES
  // //
  // =========================================================================
  // bool isWallDriveActive() const {
  //   return wallDriveState_ != WallDriveState::Idle;
  // }

  // void stopWallDrive() {
  //   setMotorsDirect(0.0, 0.0);
  //   wallDriveState_ = WallDriveState::Idle;
  //   std::cout << "Wall drive manually stopped." << std::endl;
  // }

  // =========================================================================
  // writeDebugReport - Generate comprehensive debug report
  // COMMENTED OUT - uses front distance functions not needed for fast drive
  // =========================================================================
  // void writeDebugReport() {
  //   if (!debugFile_.is_open())
  //     return;
  //
  //   double odomDist = getOdometryDistance();
  //   double frontDist = getFrontDistance();
  //   double d0 = sensors_ ? sensors_->getDistance(0) : 0.0;
  //   double d7 = sensors_ ? sensors_->getDistance(7) : 0.0;
  //
  //   debugFile_ << "\n";
  //   debugFile_
  //       << "╔══════════════════════════════════════════════════════════╗\n";
  //   debugFile_
  //       << "║           TESTING FUNCTIONS DEBUG REPORT                 ║\n";
  //   debugFile_
  //       << "╚══════════════════════════════════════════════════════════╝\n";
  //   debugFile_ << "\n";
  //
  //   debugFile_ << "=== CONFIGURATION ===\n";
  //   debugFile_ << std::fixed << std::setprecision(5);
  //   debugFile_ << "  Target stop distance:    " << TARGET_STOP_DIST << "
  //   m\n"; debugFile_ << "  Position tolerance:      " << POSITION_TOLERANCE
  //   << " m\n"; debugFile_ << "  Settle time required:    " << SETTLE_TIME <<
  //   " s\n"; debugFile_ << "  Wall close threshold:    " <<
  //   WALL_CLOSE_THRESHOLD
  //              << " m\n";
  //   debugFile_ << "  LongRun->ShortRun at:    " << TRANSITION_THRESHOLD
  //              << " m\n";
  //   debugFile_ << "  FineTune threshold:      " << FINE_TUNE_THRESHOLD
  //              << " m\n";
  //   debugFile_ << "  Max speed:               " << MAX_SPEED << " rad/s\n";
  //   debugFile_ << "  Short run speed:         " << SHORT_RUN_SPEED
  //              << " rad/s\n";
  //   debugFile_ << "  Ramp rate:               " << RAMP_RATE << " rad/s²\n";
  //   debugFile_ << "  Cos(13°) correction:     " << COS_FRONT_ANGLE << "\n";
  //   debugFile_ << "\n";
  //
  //   debugFile_ << "=== CURRENT STATE ===\n";
  //   debugFile_ << "  State: ";
  //   switch (wallDriveState_) {
  //   case WallDriveState::Idle:
  //     debugFile_ << "IDLE";
  //     break;
  //   case WallDriveState::ShortRun:
  //     debugFile_ << "SHORT_RUN";
  //     break;
  //   case WallDriveState::LongRunAccel:
  //     debugFile_ << "LONG_RUN_ACCEL";
  //     break;
  //   case WallDriveState::LongRunCruise:
  //     debugFile_ << "LONG_RUN_CRUISE";
  //     break;
  //   case WallDriveState::LongRunDecel:
  //     debugFile_ << "LONG_RUN_DECEL";
  //     break;
  //   case WallDriveState::FineSettle:
  //     debugFile_ << "FINE_SETTLE";
  //     break;
  //   }
  //   debugFile_ << "\n";
  //   debugFile_ << "  Current speed:           " << currentSpeed_ << "
  //   rad/s\n"; debugFile_ << "  Odometry distance:       " << odomDist << "
  //   m\n"; debugFile_ << "  Target odom distance:    " << targetOdomDist_ << "
  //   m\n"; debugFile_ << "  Settle time accumulated: " << settleTime_ << "
  //   s\n"; debugFile_ << "\n";
  //
  //   debugFile_ << "=== SENSOR READINGS ===\n";
  //   debugFile_ << "  ps0 raw:                 " << d0 << " m\n";
  //   debugFile_ << "  ps7 raw:                 " << d7 << " m\n";
  //   debugFile_ << "  Front (corrected):       " << frontDist << " m\n";
  //   debugFile_ << "  Error from target:       "
  //              << (frontDist - TARGET_STOP_DIST) << " m\n";
  //   debugFile_ << "\n";
  //
  //   debugFile_ << "=== ENCODER DATA ===\n";
  //   if (sensors_) {
  //     debugFile_ << "  Left encoder:            " <<
  //     sensors_->getLeftEncoder()
  //                << " rad\n";
  //     debugFile_ << "  Right encoder:           " <<
  //     sensors_->getRightEncoder()
  //                << " rad\n";
  //     debugFile_ << "  Start left:              " << startEncL_ << " rad\n";
  //     debugFile_ << "  Start right:             " << startEncR_ << " rad\n";
  //   }
  //   debugFile_ << "\n";
  //
  //   debugFile_
  //       <<
  //       "══════════════════════════════════════════════════════════════\n";
  //   debugFile_.flush();
  //
  //   std::cout << "Debug report written to testing_debugs.txt" << std::endl;
  // }
};

} // namespace Testing

#endif // TESTING_FUNCTIONS_H
