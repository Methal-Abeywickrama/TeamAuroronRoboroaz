#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <webots/Motor.hpp>

// Future Sensor Fusion Integration
#include "sensing.h"
// Note: We will use sensor fusion of IMU and IR sensors in both sides for
// future in this motion controller file.

namespace Motion {

struct MotionProfile {
  double Kp;
  double Ki;
  double Kd;
  double k_wall; // Centering Gain
  std::string name;
};

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
  Sensor::Sensing *sensors;

  // --- 2. THE BRAIN ---
  // PID Controller for Distance (Drive Control)
  PID distPID;
  // PID Controller for Heading (Rotation & Correction)
  PID anglePID;

  // --- 3. PHYSICAL CONSTANTS (The Calibration Targets) ---
  // Wheel Radius: Critical for Odometry (Rad -> Meters conversion)
  // E-puck standard is approx 20.5mm or 0.0205m.
  double WHEEL_RADIUS = 0.02001;

  // --- 4. STATE MEMORY ---
  enum State { IDLE, DRIVING, ROTATING };
  State currentState;

  double targetDistance; // how far we WANT to go (Meters)
  double targetHeading;  // where we WANT to face (Radians)
  double startEncLeft;   // Encoder reading when we started
  double startEncRight;  // Encoder reading when we started

  // SNAP COOLDOWN
  double timeSinceLastSnap;
  const double SNAP_COOLDOWN = 2.0; // Seconds

  // Dynamic PID State
  double activeKWall;

  // --- 5. HELPERS ---

  // Helper: Rounds the current yaw to the nearest 90 degrees (1.57 radians)
  // Examples:
  // Input 0.1 rad  -> Returns 0.0
  // Input 1.6 rad  -> Returns 1.57 (PI/2)
  double getNearestGridHeading(double currentYaw) {
    const double GRID_STEP = M_PI / 2.0; // 90 degrees
    return std::round(currentYaw / GRID_STEP) * GRID_STEP;
  }

  void tryGridSnap() {
    // Cooldown Check
    if (timeSinceLastSnap < SNAP_COOLDOWN)
      return;

    // 1. Get Wall Data
    // 5=Left, 2=Right in Sensing.h
    double dLeft = sensors->getDistance(5);
    double dRight = sensors->getDistance(2);

    // 2. Define Safety Thresholds
    // We only snap if we are firmly INSIDE a corridor.
    // Wall must be closer than 20cm (0.2m)
    bool solidCorridor = (dLeft < 0.20 && dRight < 0.20);

    if (solidCorridor) {
      double currentYaw = sensors->getYaw();
      double idealYaw = getNearestGridHeading(currentYaw);

      // 3. Calculate Drift Error
      double drift = std::abs(currentYaw - idealYaw);

      // 4. THE SNAP CONDITIONS (The "Goldilocks" Zone)
      // A. Drift must be noticeable (> 0.5 degrees approx 0.008 rad)
      // B. Drift must NOT be huge (< 20 degrees approx 0.35 rad)
      if (drift > 0.008 && drift < 0.35) {

        // 5. EXECUTE THE OVERWRITE
        sensors->overwriteYaw(idealYaw);

        // 6. Reset PID Memory (CRITICAL!)
        // If we don't do this, the Derivative term sees a massive "Jump"
        // in error and slams the brakes. We tell it: "Relax, error is 0 now."
        anglePID.reset();

        std::cout << "SNAP: Drift " << drift << " fixed to " << idealYaw
                  << std::endl;
        timeSinceLastSnap = 0.0; // Reset Cooldown
      }
    }
  }

public:
  // Constructor
  // Initializes motors in Velocity Mode and configures the PID.
  MotionController(webots::Robot *robot, Sensor::Sensing *s)
      : sensors(s),
        // Initialize PID with a default config first.
        // We apply the specific tuning in the body below to respect strict
        // struct rules.
        distPID(PIDConfig()), anglePID(PIDConfig()) {
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

    distPID.setConfig(driveConfig);

    // 4. Initial Default Profile (Parking/Safe)
    activeKWall = 3.3;    // Default
    PIDConfig turnConfig; // Temp default setup
    turnConfig.Kp = 2.1;
    turnConfig.Kd = 0.3;
    anglePID.setConfig(turnConfig);

    currentState = IDLE;
    targetDistance = 0.0;
    targetHeading = 0.0;
    startEncLeft = 0.0;
    startEncRight = 0.0;
    timeSinceLastSnap = 0.0;
  }

  // --- COMMANDS ---

  // Call this ONCE to start the action
  void moveForward(double meters) {
    // 1. Reset the Brain
    distPID.reset();
    anglePID.reset(); // Reset angle PID too to start clean

    // 2. Take the Snapshot (Relative Odometry Zero)
    if (sensors) {
      startEncLeft = sensors->getLeftEncoder();
      startEncRight = sensors->getRightEncoder();
      // Lock onto the CURRENT heading as the target
      targetHeading = sensors->getYaw();
    }

    // 3. Set Targets
    targetDistance = meters;
    currentState = DRIVING;

    std::cout << "CMD: Moving Forward " << meters << "m, Holding Heading "
              << targetHeading << std::endl;
  }

  // Rotate LEFT by N * 90 degrees
  void turnLeft(int steps) {
    if (!sensors)
      return;
    double stepSize = 1.5708; // PI/2 with 4 decimal precision
    targetHeading = sensors->getYaw() + (steps * stepSize);

    // Normalize Target immediatley to keep it clean
    while (targetHeading > M_PI)
      targetHeading -= 2.0 * M_PI;
    while (targetHeading < -M_PI)
      targetHeading += 2.0 * M_PI;

    currentState = ROTATING;
    anglePID.reset();

    // Apply Rotation Profile Immediatley
    applyProfile(getRotationProfile());

    std::cout << "CMD: Turning Left " << steps
              << " steps. New Head: " << targetHeading << std::endl;
  }

  // Rotate RIGHT by N * 90 degrees
  void turnRight(int steps) {
    if (!sensors)
      return;
    double stepSize = 1.5708;
    targetHeading = sensors->getYaw() - (steps * stepSize);

    while (targetHeading > M_PI)
      targetHeading -= 2.0 * M_PI;
    while (targetHeading < -M_PI)
      targetHeading += 2.0 * M_PI;

    currentState = ROTATING;
    anglePID.reset();

    // Apply Rotation Profile Immediatley
    applyProfile(getRotationProfile());

    std::cout << "CMD: Turning Right " << steps
              << " steps. New Head: " << targetHeading << std::endl;
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

    // --- DRIVING STATE: PID Distance Control + Heading Correction ---
    if (currentState == DRIVING) {
      // 1. Get Current Sensor Data (Odometry Read)
      double currL = sensors->getLeftEncoder();
      double currR = sensors->getRightEncoder();
      double currentYaw = sensors->getYaw();

      // 2. Calculate Distance Traveled
      double distL = (currL - startEncLeft) * WHEEL_RADIUS;
      double distR = (currR - startEncRight) * WHEEL_RADIUS;
      double currentAvgDist = (distL + distR) / 2.0;

      // 3. Distance PID
      PIDResult distRes = distPID.calculate(targetDistance, currentAvgDist, dt);
      double distSpeed = distRes.output;

      // --- DYNAMIC PROFILE SWITCHING ---
      // Update PID and Wall Gain based on current speed
      MotionProfile prof = getDynamicProfile(distSpeed);
      applyProfile(prof);

      // 4. Heading PID (Keep straight!)
      // A. "Where are we?"
      // A. "Where are we?"
      // (currentYaw already defined above)

      // B. "What is the error?"
      double errorYaw = targetHeading - currentYaw;

      // C. Normalize the error (The Pacman Fix)
      // Example: Target 3.14, Current -3.14 -> Error 6.28 -> Norm 0.0
      while (errorYaw > M_PI)
        errorYaw -= 2.0 * M_PI;
      while (errorYaw < -M_PI)
        errorYaw += 2.0 * M_PI;

      // D. Calculate Correction
      // We want to drive Error to 0.
      // PID: Target=0, Current=-errorYaw.
      // (because PID calculates target - current -> 0 - (-error) = +error)
      PIDResult angleRes = anglePID.calculate(0, -errorYaw, dt);
      double turnCorrection = angleRes.output;

      // --- LAYER 4: PROPORTIONAL WALL CENTERING ---
      // "Soft Force Field" - Balances both walls simultaneously.

      // --- LAYER 4: PROPORTIONAL WALL CENTERING ---
      // "Soft Force Field" - Balances both walls simultaneously.

      const double WALL_THRESH = 0.125; // 0.125m (Half of 0.25m tile)
      // Use Dynamic Gain from Profile
      const double K_WALL = activeKWall;

      double dLeft = sensors->getDistance(5);
      double dRight = sensors->getDistance(2);

      double nudge = 0.0;

      // Left Wall pushes us Right (Negative bias)
      if (dLeft < WALL_THRESH) {
        double push = (WALL_THRESH - dLeft); // Magnitude of danger
        nudge -= push * K_WALL;
      }

      // Right Wall pushes us Left (Positive bias)
      if (dRight < WALL_THRESH) {
        double push = (WALL_THRESH - dRight);
        nudge += push * K_WALL;
      }

      turnCorrection += nudge;

      // 5. Motor Mixing
      // Left = Base - Turn, Right = Base + Turn (or vice versa depending on
      // sign) If Turn > 0 (Need to turn Left), Right motor speeds up.
      double leftSpeed = distSpeed - turnCorrection;
      double rightSpeed = distSpeed + turnCorrection;

      // 6. Clamp to Max Speed (Prevent Webots Warnings)
      double max_speed = 6.28;
      if (leftSpeed > max_speed)
        leftSpeed = max_speed;
      if (leftSpeed < -max_speed)
        leftSpeed = -max_speed;
      if (rightSpeed > max_speed)
        rightSpeed = max_speed;
      if (rightSpeed < -max_speed)
        rightSpeed = -max_speed;

      // 7. Apply Command
      leftMotor->setVelocity(leftSpeed);
      rightMotor->setVelocity(rightSpeed);

      // 7. Grid Snap Check (Re-enabled with Cooldown)
      timeSinceLastSnap += dt;
      tryGridSnap();

      // 8. Completion Check
      double error = std::abs(targetDistance - currentAvgDist);
      if (error < 0.01 && std::abs(distSpeed) < 0.1) {
        stop();
      }
    }

    // --- ROTATING STATE ---
    if (currentState == ROTATING) {
      double currentYaw = sensors->getYaw();
      double errorYaw = targetHeading - currentYaw;

      while (errorYaw > M_PI)
        errorYaw -= 2.0 * M_PI;
      while (errorYaw < -M_PI)
        errorYaw += 2.0 * M_PI;

      // PID Correction
      PIDResult turnRes = anglePID.calculate(0, -errorYaw, dt);
      double turnSpeed = turnRes.output;

      // Apply (Spin in place)
      leftMotor->setVelocity(-turnSpeed);
      rightMotor->setVelocity(turnSpeed);

      // Completion Check
      if (std::abs(errorYaw) < 0.005 && std::abs(turnSpeed) < 0.1) {
        stop();
        std::cout << "TURN COMPLETE." << std::endl;
      }
    }
  }

  // -------------------------------------------------------------------------
  // HELPERS & PROFILES
  // -------------------------------------------------------------------------

  void applyProfile(MotionProfile p) {
    // Update Angle PID Config
    PIDConfig c = anglePID.getConfig();
    c.Kp = p.Kp;
    c.Ki = p.Ki;
    c.Kd = p.Kd;
    anglePID.setConfig(c);

    // Update Wall Centering Gain
    activeKWall = p.k_wall;
  }

  MotionProfile getRotationProfile() {
    return {1.5, 0.0, 0.8, 0.0, "Rotation"};
  }

  // Returns the correct profile based on absolute speed (rad/s)
  MotionProfile getDynamicProfile(double speed) {
    double s = std::abs(speed);
    if (s > 4.0)
      return {2.1, 0.0, 0.3, 3.3, "Turbo"};
    if (s > 2.0)
      return {1.8, 0.0, 0.5, 2.5, "Cruise"};
    if (s > 0.5)
      return {1.5, 0.0, 0.8, 1.5, "Approach"};
    return {1.0, 0.0, 0.1, 0.0, "Parking"};
  }

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
