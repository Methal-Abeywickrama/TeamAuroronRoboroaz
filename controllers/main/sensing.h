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
  const double INTERFERENCE_FACTOR;
  const double POWER_COEFF_A;
  const double POWER_COEFF_B;
  const double CLOSE_COEFF_A;
  const double CLOSE_COEFF_B;

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

  bool checkRegionForGreen(int startX, int endX, int startY, int endY) {
        if (!camera) return false;
        const unsigned char* image = camera->getImage();
        if (!image) return false;

        int w = camera->getWidth();
        int h = camera->getHeight();

        long r = 0, g = 0, b = 0;
        int count = 0;

        // 1. Iterate through the requested box
        for (int x = startX; x < endX; x++) {
            for (int y = startY; y < endY; y++) {
                // Bounds safety check
                if (x >= 0 && x < w && y >= 0 && y < h) {
                    r += webots::Camera::imageGetRed(image, w, x, y);
                    g += webots::Camera::imageGetGreen(image, w, x, y);
                    b += webots::Camera::imageGetBlue(image, w, x, y);
                    count++;
                }
            }
        }

        if (count == 0) return false;

        // 2. Average the colors
        r /= count;
        g /= count;
        b /= count;

        // 3. Green Detection Logic (Same threshold as your other functions)
        // Green must be significantly brighter than Red and Blue
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
  Sensing(webots::Robot *robot, int timeStep) :

    INTERFERENCE_FACTOR(0.05),    // Removes Light Noise
    POWER_COEFF_A(3.7),           // Zone 1 Scale
    POWER_COEFF_B(-0.5),          // Zone 1 Slope (Standard Physics)
    CLOSE_COEFF_A(600000.0),      // Zone 2 Scale (Force fit for close range)
    CLOSE_COEFF_B(-2.1)
    {
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
    // for (int i = 0; i < 8; i++) {
    //   if (i < distanceSensors.size()) {
    //     double raw = distanceSensors[i]->getValue();

    //     double meters;
    //     if (raw < 80.0) {
    //       meters = 2.0; // Far away / Infinity
    //     } else {
    //       meters = 0.185 * pow(raw / 100.0, -0.70);
    //     }

    //     // Filter: 60% New, 40% Old
    //     cached_dist_meters[i] = (0.6 * meters) + (0.4 * cached_dist_meters[i]);
    //   }
    // }
    // C. Update Distance Sensors (Power Law Linearization) for side and back sensors
    for (int i = 0; i < 8; i++) {
            if (i < (int)distanceSensors.size()) {
                double rawIR = distanceSensors[i]->getValue();
                
                double ambientLight = 0.0;
                if (i < (int)lightSensors.size()) {
                    ambientLight = lightSensors[i]->getValue();
                }

                // DYNAMIC CANCELLATION (New k = 0.05)
                double trueSignal = rawIR - (ambientLight * INTERFERENCE_FACTOR);
                
                // CRITICAL FIX:
                // If we subtract too much, signal goes negative. 
                // We clamp it to 20.0 (minimum valid noise floor) so math doesn't break.
                if (trueSignal < 20.0) trueSignal = 20.0;

                // TWO-ZONE CONVERSION
                double meters = 0.0;
                
                // Transition point at raw 1800
                if (trueSignal < 1800.0) {
                    meters = POWER_COEFF_A * pow(trueSignal, POWER_COEFF_B);
                } else {
                    meters = CLOSE_COEFF_A * pow(trueSignal, CLOSE_COEFF_B);
                }

                cached_dist_meters[i] = (0.6 * meters) + (0.4 * cached_dist_meters[i]);
            }
        }

    // D. Update Vision
    cached_floor_color = processVision();
  }
    // E. Distance calculation for front sensors

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
    int ids[2] = {0, 7}; 
    double distances[2] = {0.0, 0.0};

    for (int k = 0; k < 2; k++) {
        int i = ids[k];
        
        // 1. Get Raw Data
        double rawIR = distanceSensors[i]->getValue();
        double ambientLight = 0.0;
        if (i < (int)lightSensors.size()) {
            ambientLight = lightSensors[i]->getValue();
        }

        // 2. Standard Cancellation (No Color Gain)
        double trueSignal = rawIR - (ambientLight * INTERFERENCE_FACTOR);
        
        // Standard noise floor clamp
        if (trueSignal < 20.0) trueSignal = 20.0;

        // 3. Conversion
        double meters = 0.0;

        if (trueSignal > 1800.0) {
             // Zone 1: Very Close (< 8cm)
             meters = CLOSE_COEFF_A * pow(trueSignal, CLOSE_COEFF_B);
        }
        else if (trueSignal > 1000.0) {
             // Zone 2: Mid Range (8cm - 12cm)
             meters = POWER_COEFF_A * pow(trueSignal, POWER_COEFF_B);
        } 
        else {
             // Zone 3: Far Range (12cm+)
             double normalized = trueSignal / 1000.0; 
             meters = 0.27 - (normalized * (0.27 - 0.117)); 
        }
        
        distances[k] = meters;
    }
    
    double avgDistance = (distances[0] + distances[1]) / 2.0;

    // 4. GEOMETRIC CORRECTION 
    // Fixes the angle of sensors 0 and 7 (~0.94 factor)
    double correctedDistance = avgDistance * 0.94; 

    // Clamp max range
    if (correctedDistance > 0.25) return 0.25;

    return correctedDistance;
  }

  // Checks if the 'Green' color dominates the camera view ( > 70% coverage)
  bool isFrontWallGreen() {
    if (!camera) return false;
    const unsigned char *image = camera->getImage();
    if (!image) return false;

    int w = camera->getWidth();
    int h = camera->getHeight();
    if (w < 1 || h < 1) return false;

    int greenPixelCount = 0;
    int totalPixels = w * h;

    // Iterate over the ENTIRE frame
    for (int x = 0; x < w; x++) {
      for (int y = 0; y < h; y++) {
        int r = webots::Camera::imageGetRed(image, w, x, y);
        int g = webots::Camera::imageGetGreen(image, w, x, y);
        int b = webots::Camera::imageGetBlue(image, w, x, y);

        // Per-Pixel Check:
        // We use a lower threshold (+20) than the average method (+40).
        // Dark pixels might be (R:15, G:40, B:15). The diff is only 25.
        // If we strictly required +40, we would miss dark green walls.
        if (g > (r + 20) && g > (b + 20)) {
           greenPixelCount++;
        }
      }
    }

    double greenRatio = (double)greenPixelCount / (double)totalPixels;

    // Optional: Print the ratio to console to help you tune the 0.70 number
    // std::cout << "Green Coverage: " << greenRatio * 100 << "%" << std::endl;

    // Threshold: 0.70 (70% of the screen must be green)
    std::cout << greenRatio << std::endl;
    return (greenRatio > 0.50);
  }

  bool is2SquaresLeftGreen() {
        int w = camera->getWidth();
        int h = camera->getHeight();

        // X: Standard 0% to 20% (Left Edge)
        int startX = 0;
        int endX = w * 0.20; 

        // Y: User said 45% to 80% (Bottom-Up)
        // In Computer Vision (Top-Down):
        // Top limit (80% up) -> Row 20% (0.20)
        // Bottom limit (45% up) -> Row 55% (0.55)
        int startY = h * 0.20; 
        int endY = h * 0.55;

        return checkRegionForGreen(startX, endX, startY, endY);
    }

    // --- NEW: Look 2 Squares Away (Right Side) ---
    bool is2SquaresRightGreen() {
        int w = camera->getWidth();
        int h = camera->getHeight();

        // X: Standard 80% to 100% (Right Edge)
        int startX = w * 0.80; 
        int endX = w;

        // Y: Same conversion as above
        int startY = h * 0.20;
        int endY = h * 0.55;

        return checkRegionForGreen(startX, endX, startY, endY);
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