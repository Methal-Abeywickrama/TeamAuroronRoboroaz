#ifndef SENSING_HPP
#define SENSING_HPP

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/LED.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

// Shared Enum for the Checkpoint Logic
enum FloorColor { COLOR_NONE, COLOR_RED, COLOR_GREEN, COLOR_CHECKERBOARD };

class Sensing {
private:
  // -------------------------------------------------------------------------
  // 1. HARDWARE POINTERS (Fast Access)
  // -------------------------------------------------------------------------
  webots::Robot *robot;
  webots::DistanceSensor *distSensors[8]; // ps0 - ps7
  webots::PositionSensor *leftEncoder;
  webots::PositionSensor *rightEncoder;
  webots::InertialUnit *imu;
  webots::Camera *camera;
  webots::LED *led;

  // -------------------------------------------------------------------------
  // 2. CACHED DATA (The Snapshot)
  // -------------------------------------------------------------------------
  // Storing processed data here guarantees synchronization across all PID
  // layers.
  double cached_dist_meters[8];
  double cached_yaw;
  double cached_enc_left;
  double cached_enc_right;
  FloorColor cached_floor_color;

  // -------------------------------------------------------------------------
  // 3. TUNING CONSTANTS (Calibration)
  // -------------------------------------------------------------------------
  // Filter Alpha: 0.6 = Moderate smoothing. Lower = smoother but laggier.
  const double FILTER_ALPHA = 0.6;

  // IR Linearization Constants (Calibrated for Standard Webots E-puck)
  // Curve fits: Dist = Scale * (Raw / 100)^Exponent
  const double CALIB_SCALE = 0.18;   // Approx max range ~20cm reliable
  const double CALIB_EXP = -0.70;    // E-puck sensor falloff curve
  const double MAX_VALID_DIST = 2.0; // Return this if sensor sees "Infinity"

  // -------------------------------------------------------------------------
  // 4. PRIVATE HELPERS (The "Translator" Logic)
  // -------------------------------------------------------------------------

  // Converts raw bits (0-4096) into real Meters for Layer 4 Fusion
  double rawToMeters(double raw_val) {
    // E-puck sensors return ~80 when looking at nothing (Infinity)
    if (raw_val < 80.0)
      return MAX_VALID_DIST;

    // Power law approximation for Sharp IR sensors
    return CALIB_SCALE * pow(raw_val / 100.0, CALIB_EXP);
  }

  // Standardizes Angle to -PI to +PI range for Layer 3 Heading Lock
  double normalizeAngle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  // Simple Vision Processor for Checkpoint Logic
  FloorColor processVision() {
    if (!camera)
      return COLOR_NONE;
    const unsigned char *image = camera->getImage();
    if (!image)
      return COLOR_NONE;

    int w = camera->getWidth();
    int h = camera->getHeight();

    // Safety Check: Ensure image is large enough for ROI
    if (w < 5 || h < 5)
      return COLOR_NONE;

    // Strategy: ROI (Region of Interest) - Center 5x5 pixels only
    // This prevents detecting checkpoints in adjacent lanes.
    long r = 0, g = 0, b = 0;
    int count = 0;
    int startX = (w / 2) - 2;
    int startY = (h / 2) - 2;

    for (int x = startX; x < startX + 5; x++) {
      for (int y = startY; y < startY + 5; y++) {
        r += webots::Camera::imageGetRed(image, w, x, y);
        g += webots::Camera::imageGetGreen(image, w, x, y);
        b += webots::Camera::imageGetBlue(image, w, x, y);
        count++;
      }
    }
    if (count == 0)
      return COLOR_NONE;
    r /= count;
    g /= count;
    b /= count;

    // Simple Color Thresholding
    if (g > r + 40 && g > b + 40)
      return COLOR_GREEN;
    if (r > g + 40 && r > b + 40)
      return COLOR_RED;

    return COLOR_NONE;
  }

public:
  // -------------------------------------------------------------------------
  // 5. CONSTRUCTOR
  // -------------------------------------------------------------------------
  Sensing(webots::Robot *r) : robot(r) {
    // Init Distance Sensors (ps0 - ps7)
    char name[4] = "ps0";
    for (int i = 0; i < 8; i++) {
      name[2] = '0' + i;
      distSensors[i] = robot->getDistanceSensor(name);
      if (distSensors[i]) {
        distSensors[i]->enable(robot->getBasicTimeStep());
      } else {
        std::cerr << "Warning: Distance Sensor " << name << " not found!"
                  << std::endl;
      }
      cached_dist_meters[i] = 0.0;
    }

    // Init Encoders
    leftEncoder = robot->getPositionSensor("left wheel sensor");
    rightEncoder = robot->getPositionSensor("right wheel sensor");
    if (leftEncoder)
      leftEncoder->enable(robot->getBasicTimeStep());
    else
      std::cerr << "Warning: Left Encoder not found!" << std::endl;

    if (rightEncoder)
      rightEncoder->enable(robot->getBasicTimeStep());
    else
      std::cerr << "Warning: Right Encoder not found!" << std::endl;

    // Init IMU
    imu = robot->getInertialUnit("inertial unit");
    if (imu) {
      imu->enable(robot->getBasicTimeStep());
    } else {
      std::cerr << "Warning: Inertial Unit not found! (Check your PROTO)"
                << std::endl;
    }

    // Init Camera (Lower update rate to save CPU)
    camera = robot->getCamera("camera");
    if (camera)
      camera->enable(64);

    // Init LED
    led = robot->getLED("led0");

    // Init Cache
    cached_yaw = 0.0;
    cached_enc_left = 0.0;
    cached_enc_right = 0.0;
    cached_floor_color = COLOR_NONE;
  }

  // -------------------------------------------------------------------------
  // 6. UPDATE LOOP (The "Snapshot" - Run once per Step)
  // -------------------------------------------------------------------------
  void update() {
    // A. Encoders (Layer 1 & 2)
    if (leftEncoder)
      cached_enc_left = leftEncoder->getValue();
    if (rightEncoder)
      cached_enc_right = rightEncoder->getValue();

    // B. IMU (Layer 3)
    // Webots returns [Roll, Pitch, Yaw]. We only need Yaw (Index 2).
    if (imu) {
      const double *rpy = imu->getRollPitchYaw();
      if (rpy && !std::isnan(rpy[2])) {
        cached_yaw = normalizeAngle(rpy[2]);
      }
    }

    // C. Distance Sensors (Layer 4)
    for (int i = 0; i < 8; i++) {
      if (!distSensors[i])
        continue;

      double raw = distSensors[i]->getValue();
      double meters = rawToMeters(raw);

      // Apply Exponential Smoothing Filter to remove Physics Noise
      cached_dist_meters[i] = (FILTER_ALPHA * meters) +
                              ((1.0 - FILTER_ALPHA) * cached_dist_meters[i]);
    }

    // D. Vision (Checkpoint Logic)
    cached_floor_color = processVision();
  }

  // -------------------------------------------------------------------------
  // 7. GETTERS (The Interface for MotionControl)
  // -------------------------------------------------------------------------

  // Returns Distance in METERS (Linearized)
  // Indices: 5=Left, 2=Right, 0=FrontLeft, 7=FrontRight (E-puck Standard)
  double getDistance(int index) {
    if (index < 0 || index > 7)
      return 99.0;
    return cached_dist_meters[index];
  }

  // Returns Yaw in Radians (-PI to +PI)
  double getYaw() { return cached_yaw; }

  // Returns Total Wheel Rotation in Radians
  double getLeftEncoder() { return cached_enc_left; }
  double getRightEncoder() { return cached_enc_right; }

  FloorColor getFloorColor() { return cached_floor_color; }

  void setLED(bool on) {
    if (led)
      led->set(on ? 1 : 0);
  }
};

#endif