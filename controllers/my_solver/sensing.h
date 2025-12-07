#ifndef SENSING_H
#define SENSING_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

// Webots Includes
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/LED.hpp>
#include <webots/LightSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

namespace Sensor {

// Shared Enum for Vision
enum FloorColor { COLOR_NONE, COLOR_RED, COLOR_GREEN, COLOR_CHECKERBOARD };

class Sensing {
private:
  // Device Pointers
  webots::Robot *robot;
  int timeStep;
  double dt_seconds;

  std::vector<webots::DistanceSensor *> distanceSensors;
  std::vector<webots::LightSensor *> lightSensors;
  std::vector<webots::LED *> leds;
  webots::Camera *camera;
  webots::PositionSensor *leftEncoder;
  webots::PositionSensor *rightEncoder;
  webots::Gyro *gyro;

  // Cache Variables (The "Snapshot")
  double cached_dist_meters[8];
  double cached_enc_left;
  double cached_enc_right;
  FloorColor cached_floor_color;

  // E-puck Physical Constants
  const double WHEEL_RADIUS = 0.0205;
  const double AXLE_LENGTH = 0.052;

  // YAW VARIABLES (Separated)
  double yaw_odometry;
  double yaw_gyro;
  double gyro_bias; // Static drift value

  // Odometry History
  double lastLeftVal;
  double lastRightVal;

  // --- HELPER: Normalize Angle (-PI to +PI) ---
  // Keeps angles clean for PID (e.g., converts 6.30 rad to 0.02 rad)
  double normalizeAngle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  // Helper: Process Vision
  FloorColor processVision() {
    if (!camera)
      return COLOR_NONE;
    const unsigned char *image = camera->getImage();
    if (!image)
      return COLOR_NONE;

    int w = camera->getWidth();
    int h = camera->getHeight();
    if (w < 5 || h < 5)
      return COLOR_NONE;

    int r = 0, g = 0, b = 0, count = 0;
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

    if (g > r + 40 && g > b + 40)
      return COLOR_GREEN;
    // Red is often detected on the walls, be careful with this threshold
    if (r > g + 40 && r > b + 40)
      return COLOR_RED;

    return COLOR_NONE;
  }

  void initSensors() {
    // 1. Distance Sensors
    std::string psNames[8] = {"ps0", "ps1", "ps2", "ps3",
                              "ps4", "ps5", "ps6", "ps7"};
    for (int i = 0; i < 8; i++) {
      webots::DistanceSensor *ds = robot->getDistanceSensor(psNames[i]);
      if (ds) {
        ds->enable(timeStep);
        distanceSensors.push_back(ds);
        cached_dist_meters[i] = 0.0;
      }
    }

    // 2. Camera
    camera = robot->getCamera("camera");
    if (camera)
      camera->enable(timeStep);

    // 3. Encoders
    leftEncoder = robot->getPositionSensor("left wheel sensor");
    rightEncoder = robot->getPositionSensor("right wheel sensor");
    if (leftEncoder)
      leftEncoder->enable(timeStep);
    if (rightEncoder)
      rightEncoder->enable(timeStep);

    // 4. Gyro
    gyro = robot->getGyro("gyro");
    if (gyro)
      gyro->enable(timeStep);

    // 5. Light Sensors (ls0 - ls7)
    std::string lsNames[8] = {"ls0", "ls1", "ls2", "ls3",
                              "ls4", "ls5", "ls6", "ls7"};
    for (int i = 0; i < 8; i++) {
      webots::LightSensor *ls = robot->getLightSensor(lsNames[i]);
      if (ls) {
        ls->enable(timeStep);
        lightSensors.push_back(ls);
      }
    }

    // 6. LEDs
    std::string lNames[10] = {"led0", "led1", "led2", "led3", "led4",
                              "led5", "led6", "led7", "led8", "led9"};
    for (int i = 0; i < 10; i++) {
      webots::LED *l = robot->getLED(lNames[i]);
      if (l)
        leds.push_back(l);
    }
  }

public:
  Sensing(webots::Robot *robot, int timeStep) {
    this->robot = robot;
    this->timeStep = timeStep;

    // Pre-calculate dt in seconds (ms -> seconds)
    this->dt_seconds = (double)timeStep / 1000.0;

    // Init logic variables
    yaw_odometry = 0.0;
    yaw_gyro = 0.0;
    gyro_bias = 0.0; // Assumed 0 until calibrated

    lastLeftVal = 0.0;
    lastRightVal = 0.0;

    initSensors();
  }

  ~Sensing() {}

  // -------------------------------------------------------------------------
  // 0. CALIBRATION (Call in main.cpp)
  // -------------------------------------------------------------------------
  // This calculates the static drift of the gyro while the robot is still.
  void calibrateGyro(int samples = 50) {
    if (!gyro)
      return;
    double sum = 0.0;
    std::cout << "Calibrating Gyro... (Keep Robot Still)" << std::endl;

    for (int i = 0; i < samples; i++) {
      robot->step(timeStep); // Advance physics
      const double *val = gyro->getValues();
      if (!std::isnan(val[2])) {
        sum += val[2]; // Z-axis
      }
    }

    gyro_bias = sum / samples;

    // Reset headings after calibration so we start fresh at 0.0
    yaw_gyro = 0.0;
    yaw_odometry = 0.0;
    std::cout << "Gyro Calibrated. Bias: " << gyro_bias << std::endl;
  }

  // -------------------------------------------------------------------------
  // 1. UPDATE LOOP (Call ONCE per step)
  // -------------------------------------------------------------------------
  void update() {
    // A. METHOD 1: Odometry (Wheel Encoders) - Kept Separate
    if (leftEncoder && rightEncoder) {
      cached_enc_left = leftEncoder->getValue();
      cached_enc_right = rightEncoder->getValue();

      double diffL = cached_enc_left - lastLeftVal;
      double diffR = cached_enc_right - lastRightVal;

      double distL = diffL * WHEEL_RADIUS;
      double distR = diffR * WHEEL_RADIUS;

      // Odometry Math
      double deltaYaw = (distR - distL) / AXLE_LENGTH;
      yaw_odometry += deltaYaw;

      // Normalize Odometry
      yaw_odometry = normalizeAngle(yaw_odometry);

      lastLeftVal = cached_enc_left;
      lastRightVal = cached_enc_right;
    }

    // B. METHOD 2: Gyroscope (Integration) - The Active One
    if (gyro) {
      const double *values = gyro->getValues();
      // Index 2 = Z-Axis (Yaw)
      if (!std::isnan(values[2])) {
        // 1. Subtract Bias
        double angularVelocity = values[2] - gyro_bias;

        // 2. Deadband (Optional: Ignore noise < 0.002 rad/s)
        if (std::abs(angularVelocity) < 0.002)
          angularVelocity = 0.0;

        // 3. Integrate
        double deltaGyro = angularVelocity * dt_seconds;
        yaw_gyro += deltaGyro;

        // 4. Normalize
        yaw_gyro = normalizeAngle(yaw_gyro);
      }
    }

    // C. Update Distance Sensors (Power Law Linearization)
    for (int i = 0; i < 8; i++) {
      if (i < distanceSensors.size()) {
        double raw = distanceSensors[i]->getValue();

        double meters;
        if (raw < 80.0) {
          meters = 2.0; // Far away / Infinity
        } else {
          meters = 0.185 * pow(raw / 100.0, -0.70);
        }

        // Filter: 60% New, 40% Old
        cached_dist_meters[i] = (0.6 * meters) + (0.4 * cached_dist_meters[i]);
      }
    }

    // D. Update Vision
    cached_floor_color = processVision();
  }

  // -------------------------------------------------------------------------
  // 2. GRID SNAP SUPPORT (Critical for MotionControl)
  // -------------------------------------------------------------------------
  // This allows the MotionController to say: "I know for a fact we are at 90
  // degrees." It overwrites the accumulated drift with the clean Grid Angle.
  void overwriteYaw(double cleanGridAngle) {
    yaw_gyro = normalizeAngle(cleanGridAngle);
    yaw_odometry =
        normalizeAngle(cleanGridAngle); // Sync both for debugging clarity
    // std::cout << "Sensing: Yaw snapped to " << cleanGridAngle << std::endl;
  }

  void resetYaw() {
    yaw_odometry = 0.0;
    yaw_gyro = 0.0;
    std::cout << "All Yaw values reset to 0.0" << std::endl;
  }

  // -------------------------------------------------------------------------
  // 3. GETTERS
  // -------------------------------------------------------------------------

  // PRIMARY GETTER: Returns Gyro Yaw (The Winner)
  double getYaw() { return yaw_gyro; }

  // Debug Getters
  double getYawOdometry() { return yaw_odometry; }
  double getYawGyro() { return yaw_gyro; } // Same as getYaw()

  double getDistance(int index) {
    if (index < 0 || index >= 8)
      return 99.0;
    return cached_dist_meters[index];
  }

  double getLeftEncoder() { return cached_enc_left; }
  double getRightEncoder() { return cached_enc_right; }
  FloorColor getFloorColor() { return cached_floor_color; }

  // Wall Helpers
  bool isWallAtFront() {
    return (getDistance(0) < 0.15) && (getDistance(7) < 0.15);
  }
  bool isWallAtBack() {
    return (getDistance(3) < 0.1) && (getDistance(4) < 0.1);
  }
  bool isWallAtLeft() { return (getDistance(5) < 0.15); }
  bool isWallAtRight() { return (getDistance(2) < 0.15); }

  double getDistanceToFrontWall() {
    double d1 = getDistance(0);
    double d2 = getDistance(7);
    if (d1 > 1.0 || d2 > 1.0)
      return 2.0;
    return (d1 + d2) / 2.0;
  }
  double getDistanceToLeftWall() { return getDistance(5); }
  double getDistanceToRightWall() { return getDistance(2); }

  void setLEDs(bool on) {
    for (auto *l : leds)
      l->set(on ? 1 : 0);
  }
};
} // namespace Sensor

#endif // SENSING_H