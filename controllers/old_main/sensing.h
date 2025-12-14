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
  double cached_raw_dist[8]; // Corrected raw values for wall detection
  double cached_enc_left;
  double cached_enc_right;
  FloorColor cached_floor_color;

  // E-puck Physical Constants
  const double WHEEL_RADIUS = 0.0205;
  const double AXLE_LENGTH = 0.052;

  // Calibration Constants for Distance Sensors (from methal_sensing.h)
  const double INTERFERENCE_FACTOR = 0.05; // Light subtraction factor
  const double POWER_COEFF_A = 3.7;        // Mid Range Scale
  const double POWER_COEFF_B = -0.5;       // Mid Range Slope
  const double CLOSE_COEFF_A = 600000.0;   // Close Range Scale
  const double CLOSE_COEFF_B = -2.1;       // Close Range Slope (Steep drop-off)

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

  // Helper: Check specific Region of Interest (ROI) for Green
  // Used by is2SquaresLeftGreen / RightGreen to detect green walls ahead
  bool checkRegionForGreen(int startX, int endX, int startY, int endY) {
    if (!camera)
      return false;
    const unsigned char *image = camera->getImage();
    if (!image)
      return false;

    int w = camera->getWidth();
    int h = camera->getHeight();
    long r = 0, g = 0, b = 0;
    int count = 0;

    for (int x = startX; x < endX; x++) {
      for (int y = startY; y < endY; y++) {
        if (x >= 0 && x < w && y >= 0 && y < h) {
          r += webots::Camera::imageGetRed(image, w, x, y);
          g += webots::Camera::imageGetGreen(image, w, x, y);
          b += webots::Camera::imageGetBlue(image, w, x, y);
          count++;
        }
      }
    }

    if (count == 0)
      return false;
    r /= count;
    g /= count;
    b /= count;

    // Green Threshold: Green must be significantly brighter than Red and Blue
    return (g > r + 30 && g > b + 30);
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

    // Initialize distance cache to max (no walls detected until updated)
    for (int i = 0; i < 8; i++) {
      cached_dist_meters[i] = 0.40; // Max distance = no wall
      cached_raw_dist[i] = 0.0;     // No signal = no wall
    }

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

    // C. Update Distance Sensors with Light Interference Correction
    // From methal_sensing.h - corrects for ambient light affecting IR sensors
    for (int i = 0; i < 8; i++) {
      if (i < distanceSensors.size()) {
        double rawIR = distanceSensors[i]->getValue();

        // Apply light interference correction
        double ambientLight = 0.0;
        if (i < (int)lightSensors.size()) {
          ambientLight = lightSensors[i]->getValue();
        }

        // Remove ambient light interference (INTERFERENCE_FACTOR = 0.05)
        double correctedRaw = rawIR - (ambientLight * 0.05);
        if (correctedRaw < 20.0)
          correctedRaw = 20.0;

        // Store corrected raw for wall detection
        cached_raw_dist[i] = correctedRaw;

        // Convert to meters using power law
        double meters;
        if (correctedRaw < 80.0) {
          meters = 2.0; // Far away / Infinity
        } else {
          meters = 0.185 * pow(correctedRaw / 100.0, -0.70);
        }

        // Filter: 60% New, 40% Old
        cached_dist_meters[i] =
            (0.75 * meters) + (0.25 * cached_dist_meters[i]);
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

  // Wall Helpers - use RAW sensor values for reliable wall detection
  // Calibration: raw=40→~0.33m, raw=60→~0.27m, raw=80→~0.22m, raw=400→~0.08m
  bool isWallAtFront() {
    // Check RAW values directly for walls at exploration distance
    double raw0 = getRawDistance(0);
    double raw7 = getRawDistance(7);
    // Threshold: raw > 30 means wall within detection range (lowered for dark
    // walls)
    return (raw0 > 30.0) || (raw7 > 30.0);
  }
  bool isWallAtBack() {
    return (getRawDistance(3) > 150.0) && (getRawDistance(4) > 150.0);
  }
  bool isWallAtLeft() {
    // PS5 is left sensor
    return getRawDistance(5) > 60.0; // Wall within ~0.27m
  }
  bool isWallAtRight() {
    // PS2 is right sensor
    return getRawDistance(2) > 60.0; // Wall within ~0.27m
  }

  // Specialized Front Distance (Averaged + Corrected) - from methal_sensing.h
  double getDistanceToFrontWall() {
    int ids[2] = {0, 7};
    double distances[2] = {0.0, 0.0};

    // Calculate raw distance for both front sensors
    for (int k = 0; k < 2; k++) {
      int i = ids[k];
      double rawIR = distanceSensors[i]->getValue();
      double ambientLight =
          (i < (int)lightSensors.size()) ? lightSensors[i]->getValue() : 0.0;

      double trueSignal = rawIR - (ambientLight * INTERFERENCE_FACTOR);
      if (trueSignal < 20.0)
        trueSignal = 20.0;

      double meters = 0.0;
      if (trueSignal > 1800.0) {
        // Zone 1: CLOSE (< 5cm, Crash Imminent)
        meters = CLOSE_COEFF_A * pow(trueSignal, CLOSE_COEFF_B);
      } else if (trueSignal > 300.0) {
        // Zone 2: FAR (5-9cm) - Max sensor detection range
        // Linear interpolation: raw=300 → 0.09m, raw=1800 → 0.05m
        double normalized = (trueSignal - 300.0) / (1800.0 - 300.0);
        meters = 0.09 - (normalized * (0.09 - 0.05));
      } else {
        // NO WALL - below detection threshold (raw <= 300)
        // Return max distance to indicate no wall detected
        meters = 0.25;
      }
      distances[k] = meters;
    }

    double avgDistance = (distances[0] + distances[1]) / 2.0;

    // Geometric Correction (Sensors angled 13 degrees)
    // cos(13°) = cos(13 * π/180) = 0.9744
    double correctedDistance = avgDistance * 0.9744;

    // Clamp max range to 9cm (max sensor range)
    // If distance > 0.09m, sensor can't reliably detect
    if (correctedDistance > 0.09)
      return 0.09;
    return correctedDistance;
  }
  double getDistanceToLeftWall() { return getDistance(5); }
  double getDistanceToRightWall() { return getDistance(2); }

  // Returns true if front wall is predominantly green (>50% coverage)
  bool isFrontWallGreen() {
    if (!camera)
      return false;
    const unsigned char *image = camera->getImage();
    if (!image)
      return false;

    int w = camera->getWidth();
    int h = camera->getHeight();
    int greenPixelCount = 0;
    int totalPixels = w * h;

    for (int x = 0; x < w; x++) {
      for (int y = 0; y < h; y++) {
        int r = webots::Camera::imageGetRed(image, w, x, y);
        int g = webots::Camera::imageGetGreen(image, w, x, y);
        int b = webots::Camera::imageGetBlue(image, w, x, y);

        // Green threshold: green channel significantly higher than red and blue
        if (g > (r + 20) && g > (b + 20)) {
          greenPixelCount++;
        }
      }
    }

    double greenRatio = (double)greenPixelCount / (double)totalPixels;
    return (greenRatio > 0.50); // 50% threshold
  }

  // Lookahead: Check for green wall on LEFT side (next cell's left)
  // Scans left 20% of camera view, middle vertical band
  bool is2SquaresLeftGreen() {
    if (!camera)
      return false;
    int w = camera->getWidth();
    int h = camera->getHeight();
    // Left 20% of screen, 20%-55% vertical band
    return checkRegionForGreen(0, (int)(w * 0.20), (int)(h * 0.20),
                               (int)(h * 0.55));
  }

  // Lookahead: Check for green wall on RIGHT side (next cell's right)
  // Scans right 20% of camera view, middle vertical band
  bool is2SquaresRightGreen() {
    if (!camera)
      return false;
    int w = camera->getWidth();
    int h = camera->getHeight();
    // Right 20% of screen, 20%-55% vertical band
    return checkRegionForGreen((int)(w * 0.80), w, (int)(h * 0.20),
                               (int)(h * 0.55));
  }

  void setLEDs(bool on) {
    for (auto *l : leds)
      l->set(on ? 1 : 0);
  }

  // Get corrected raw sensor value (with light interference removed)
  // Used for wall detection thresholds
  double getRawDistance(int index) {
    if (index < 0 || index >= 8)
      return 0.0;
    return cached_raw_dist[index];
  }

  // Get truly raw sensor value (no correction) for debugging
  double getUncorrectedRawDistance(int index) {
    if (index < 0 || index >= (int)distanceSensors.size())
      return -1.0;
    return distanceSensors[index]->getValue();
  }

  // DEBUG: Print all raw sensor values
  void printRawSensors() {
    std::cout << "RAW SENSORS: ";
    for (int i = 0; i < 8; i++) {
      if (i < (int)distanceSensors.size()) {
        std::cout << "ps" << i << "=" << distanceSensors[i]->getValue() << " ";
      } else {
        std::cout << "ps" << i << "=N/A ";
      }
    }
    std::cout << std::endl;
  }
};
} // namespace Sensor

#endif // SENSING_H