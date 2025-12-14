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
  // --------------------------------------------------------------------------
  // 1. HARDWARE POINTERS
  // --------------------------------------------------------------------------
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

  // --------------------------------------------------------------------------
  // 2. INTERNAL STATE (Cache)
  // --------------------------------------------------------------------------
  double cached_dist_meters[8];
  double cached_enc_left;
  double cached_enc_right;
  FloorColor cached_floor_color;

  // Yaw Tracking
  double yaw_odometry;
  double yaw_gyro;
  double gyro_bias; // Static drift offset

  // Odometry History (for calculating delta)
  double lastLeftVal;
  double lastRightVal;

  // --------------------------------------------------------------------------
  // 3. PHYSICAL CONSTANTS (In-Class Initialization C++11)
  // --------------------------------------------------------------------------
  const double WHEEL_RADIUS;
  const double AXLE_LENGTH;

  // Calibration Constants for Distance Sensors
  const double INTERFERENCE_FACTOR; // Light subtraction factor
  const double POWER_COEFF_A;       // Mid Range Scale
  const double POWER_COEFF_B;       // Mid Range Slope
  const double CLOSE_COEFF_A;       // Close Range Scale
  const double CLOSE_COEFF_B;       // Close Range Slope (Steep drop-off)

  // --------------------------------------------------------------------------
  // 4. HELPER FUNCTIONS
  // --------------------------------------------------------------------------

  // Normalize Angle: Keeps angles between -PI and +PI
  double normalizeAngle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  // Vision Helper: Check specific Region of Interest (ROI) for Green
  // Used by is2SquaresLeftGreen / RightGreen to scan specific parts of the
  // image
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

  // Vision Helper: Process Floor Color (Center of Image)
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

    // 2. Light Sensors (for interference cancellation)
    std::string lsNames[8] = {"ls0", "ls1", "ls2", "ls3",
                              "ls4", "ls5", "ls6", "ls7"};
    for (int i = 0; i < 8; i++) {
      webots::LightSensor *ls = robot->getLightSensor(lsNames[i]);
      if (ls) {
        ls->enable(timeStep);
        lightSensors.push_back(ls);
      }
    }

    // 3. Other Devices
    camera = robot->getCamera("camera");
    if (camera)
      camera->enable(timeStep);

    leftEncoder = robot->getPositionSensor("left wheel sensor");
    rightEncoder = robot->getPositionSensor("right wheel sensor");
    if (leftEncoder)
      leftEncoder->enable(timeStep);
    if (rightEncoder)
      rightEncoder->enable(timeStep);

    gyro = robot->getGyro("gyro");
    if (gyro)
      gyro->enable(timeStep);

    std::string lNames[10] = {"led0", "led1", "led2", "led3", "led4",
                              "led5", "led6", "led7", "led8", "led9"};
    for (int i = 0; i < 10; i++) {
      webots::LED *l = robot->getLED(lNames[i]);
      if (l)
        leds.push_back(l);
    }
  }

public:
  // --------------------------------------------------------------------------
  // CONSTRUCTOR
  // --------------------------------------------------------------------------
  Sensing(webots::Robot *robot, int timeStep)
      : // Physical Constants Initialization
        WHEEL_RADIUS(0.0205), AXLE_LENGTH(0.052),
        INTERFERENCE_FACTOR(0.05), // Removes Light Noise
        POWER_COEFF_A(3.7),        // Mid Range Scale
        POWER_COEFF_B(-0.5),       // Mid Range Slope
        CLOSE_COEFF_A(600000.0),   // Close Range Scale
        CLOSE_COEFF_B(-2.1)        // Close Range Slope (Steep drop-off)
  {
    this->robot = robot;
    this->timeStep = timeStep;
    this->dt_seconds = (double)timeStep / 1000.0;

    yaw_odometry = 0.0;
    yaw_gyro = 0.0;
    gyro_bias = 0.0;
    lastLeftVal = 0.0;
    lastRightVal = 0.0;

    initSensors();
  }

  ~Sensing() {}

  // --------------------------------------------------------------------------
  // MAIN UPDATE LOOP (Call once per step)
  // --------------------------------------------------------------------------
  void update() {
    // A. Odometry Update
    if (leftEncoder && rightEncoder) {
      cached_enc_left = leftEncoder->getValue();
      cached_enc_right = rightEncoder->getValue();

      double diffL = cached_enc_left - lastLeftVal;
      double diffR = cached_enc_right - lastRightVal;
      double distL = diffL * WHEEL_RADIUS;
      double distR = diffR * WHEEL_RADIUS;

      double deltaYaw = (distR - distL) / AXLE_LENGTH;
      yaw_odometry = normalizeAngle(yaw_odometry + deltaYaw);

      lastLeftVal = cached_enc_left;
      lastRightVal = cached_enc_right;
    }

    // B. Gyro Integration
    if (gyro) {
      const double *values = gyro->getValues();
      if (!std::isnan(values[2])) {
        double velocity = values[2] - gyro_bias;
        if (std::abs(velocity) < 0.002)
          velocity = 0.0; // Deadband
        yaw_gyro = normalizeAngle(yaw_gyro + (velocity * dt_seconds));
      }
    }

    // C. Distance Sensors (Hybrid Linearization)
    for (int i = 0; i < 8; i++) {
      if (i < (int)distanceSensors.size()) {
        double rawIR = distanceSensors[i]->getValue();
        double ambientLight = 0.0;
        if (i < (int)lightSensors.size()) {
          ambientLight = lightSensors[i]->getValue();
        }

        // 1. Remove Ambient Noise
        double trueSignal = rawIR - (ambientLight * INTERFERENCE_FACTOR);
        if (trueSignal < 20.0)
          trueSignal = 20.0;

        // 2. Hybrid Distance Calculation
        double meters = 0.0;
        if (trueSignal > 1800.0) {
          // Zone 1: Very Close (Crash Imminent)
          meters = CLOSE_COEFF_A * pow(trueSignal, CLOSE_COEFF_B);
        } else if (trueSignal > 1000.0) {
          // Zone 2: Mid Range (Standard Physics)
          meters = POWER_COEFF_A * pow(trueSignal, POWER_COEFF_B);
        } else {
          // Zone 3: Far Range (Linearized to prevent jumps)
          double normalized = trueSignal / 1000.0;
          meters = 0.27 - (normalized * (0.27 - 0.117));
        }

        // 3. Filter
        cached_dist_meters[i] = (0.6 * meters) + (0.4 * cached_dist_meters[i]);
      }
    }

    // D. Vision
    cached_floor_color = processVision();
  }

  // --------------------------------------------------------------------------
  // SENSOR GETTERS
  // --------------------------------------------------------------------------

  // Basic Distance
  double getDistance(int index) {
    if (index < 0 || index >= 8)
      return 99.0;
    return cached_dist_meters[index];
  }

  // Specialized Front Distance (Averaged + Corrected)
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
        meters = CLOSE_COEFF_A * pow(trueSignal, CLOSE_COEFF_B);
      } else if (trueSignal > 1000.0) {
        meters = POWER_COEFF_A * pow(trueSignal, POWER_COEFF_B);
      } else {
        double normalized = trueSignal / 1000.0;
        meters = 0.27 - (normalized * (0.27 - 0.117));
      }
      distances[k] = meters;
    }

    double avgDistance = (distances[0] + distances[1]) / 2.0;

    // Geometric Correction (Sensors angled 20 degrees)
    double correctedDistance = avgDistance * 0.94;

    // Clamp max range to 25cm
    if (correctedDistance > 0.25)
      return 0.25;
    return correctedDistance;
  }

  double getDistanceToLeftWall() { return getDistance(5); }
  double getDistanceToRightWall() { return getDistance(2); }

  // --------------------------------------------------------------------------
  // VISION LOGIC
  // --------------------------------------------------------------------------

  // 1. Is the Front Wall Green? (>50% coverage)
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

        // Lower threshold (+20) for darker walls
        if (g > (r + 20) && g > (b + 20)) {
          greenPixelCount++;
        }
      }
    }

    double greenRatio = (double)greenPixelCount / (double)totalPixels;
    // std::cout << "Green Ratio: " << greenRatio << std::endl;
    return (greenRatio > 0.50);
  }

  // 2. Is the floor Green?
  bool isWallGreen() { return (processVision() == COLOR_GREEN); }

  // 3. Look Ahead Logic (2 Squares Away)
  bool is2SquaresLeftGreen() {
    int w = camera->getWidth();
    int h = camera->getHeight();
    // Left 20% of screen, Bottom-Middle Vertical Band
    return checkRegionForGreen(0, w * 0.20, h * 0.20, h * 0.55);
  }

  bool is2SquaresRightGreen() {
    int w = camera->getWidth();
    int h = camera->getHeight();
    // Right 20% of screen, Bottom-Middle Vertical Band
    return checkRegionForGreen(w * 0.80, w, h * 0.20, h * 0.55);
  }

  // --------------------------------------------------------------------------
  // UTILITIES
  // --------------------------------------------------------------------------
  void calibrateGyro(int samples = 50) {
    if (!gyro)
      return;
    double sum = 0.0;
    std::cout << "Calibrating Gyro... (Keep Still)" << std::endl;
    for (int i = 0; i < samples; i++) {
      robot->step(timeStep);
      const double *val = gyro->getValues();
      if (!std::isnan(val[2]))
        sum += val[2];
    }
    gyro_bias = sum / samples;
    yaw_gyro = 0.0;
    yaw_odometry = 0.0;
    std::cout << "Calibration Done. Bias: " << gyro_bias << std::endl;
  }

  void overwriteYaw(double cleanGridAngle) {
    yaw_gyro = normalizeAngle(cleanGridAngle);
    yaw_odometry = normalizeAngle(cleanGridAngle);
  }

  void resetYaw() {
    yaw_odometry = 0.0;
    yaw_gyro = 0.0;
  }

  // Getters
  double getYaw() { return yaw_gyro; }
  double getYawOdometry() { return yaw_odometry; }
  double getYawGyro() { return yaw_gyro; }
  double getLeftEncoder() { return cached_enc_left; }
  double getRightEncoder() { return cached_enc_right; }
  FloorColor getFloorColor() { return cached_floor_color; }

  void setLEDs(bool on) {
    for (auto *l : leds)
      l->set(on ? 1 : 0);
  }
};
} // namespace Sensor

#endif // SENSING_H