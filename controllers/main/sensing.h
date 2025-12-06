// #ifndef SENSING_H
// #define SENSING_H

// #include <string>
// #include <vector>
// #include <webots/Accelerometer.hpp>
// #include <webots/Camera.hpp>
// #include <webots/DistanceSensor.hpp>
// #include <webots/LightSensor.hpp>
// #include <webots/Robot.hpp>


// namespace Sensor {

// class Sensing {
// public:
//   Sensing(webots::Robot *robot, int timeStep) {
//     this->robot = robot;
//     this->timeStep = timeStep;
//     initSensors();
//   }
//   bool isWallAtFront() {
//     if (isWallDetected(0) && isWallDetected(7)) return true;
//     return false;
//   }
//   bool isWallAtBack() {
//     if (isWallDetected(3) && isWallDetected(4)) return true;
//     return false;
//   }
//   bool isWallAtLeft() {
//     if (isWallDetected(5)) return true;
//     return false;
//   }
//   bool isWallAtRight() {
//     if (isWallDetected(2)) return true;
//     return false;
//   }



//   bool isWallDetected(int sensorIndex) {
//         if (sensorIndex < 0 || sensorIndex >= distanceSensors.size()) return false;
        
//         double rawValue = distanceSensors[sensorIndex]->getValue();
//         std::cout << rawValue << std::endl;
//         // TUNING PARAMETER: You decide this number!
//         // If value > 100, we consider it a wall.
//         double threshold = 150.0; 

//         return rawValue > threshold;
//   }
//   ~Sensing() {}

//   // Add methods to get sensor values here
//   // double getDistanceSensorValue(int index);

// private:
//   webots::Robot *robot;
//   int timeStep;

//   // E-puck has 8 distance sensors usually named ps0-ps7
//   std::vector<webots::DistanceSensor *> distanceSensors;
//   // E-puck has 8 light sensors usually named ls0-ls7
//   std::vector<webots::LightSensor *> lightSensors;

//   webots::Accelerometer *accelerometer;
//   webots::Camera *camera;

//   void initSensors() {
//     // Initialize Distance Sensors
//     char psNames[8][4] = {"ps0", "ps1", "ps2", "ps3",
//                           "ps4", "ps5", "ps6", "ps7"};
//     std::cout << "helklo" << std::endl;

//     for (int i = 0; i < 8; i++) {
//       webots::DistanceSensor *ds = robot->getDistanceSensor(psNames[i]);
//       if (ds) {
//         // std::cout << "distance sensor found" << std::endl;
//         // std::cout << psNames[i] << std::endl;
//         ds->enable(timeStep);
//         distanceSensors.push_back(ds);
//       }
//     }

//     // Initialize Light Sensors
//     char lsNames[8][4] = {"ls0", "ls1", "ls2", "ls3",
//                           "ls4", "ls5", "ls6", "ls7"};

//     for (int i = 0; i < 8; i++) {
//       webots::LightSensor *ls = robot->getLightSensor(lsNames[i]);
//       if (ls) {
//         // std::cout << "light sensor found" << std::endl;
//         // std::cout << lsNames[i] << std::endl;
//         ls->enable(timeStep);
//         lightSensors.push_back(ls);
//       }
//     }

//     // Initialize Accelerometer
//     accelerometer = robot->getAccelerometer("accelerometer");
//     if (accelerometer) {
//       // std::cout << "accelerometer found" << std::endl;
//       accelerometer->enable(timeStep);
//     }

//     // Initialize Camera
//     camera = robot->getCamera("camera");
//     if (camera) {
//       // std::cout << "camera found" << std::endl;
//       camera->enable(timeStep);
//     }
//   }
// };
// } // namespace Sensor

// #endif // SENSING_H


#ifndef SENSING_H
#define SENSING_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

// Webots Includes
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/InertialUnit.hpp>    // Added for rotation (IMU)
#include <webots/PositionSensor.hpp>  // Added for wheel encoders
#include <webots/LED.hpp>             // Added for LEDs
#include <cmath>

namespace Sensor {

// Shared Enum for Vision
enum FloorColor { COLOR_NONE, COLOR_RED, COLOR_GREEN, COLOR_CHECKERBOARD };

class Sensing {
public:
    Sensing(webots::Robot *robot, int timeStep) {
        this->robot = robot;
        this->timeStep = timeStep;
        initSensors();

        // Initialize math variables
        currentYaw = 0.0;
        lastLeftVal = 0.0;
        lastRightVal = 0.0;
    }

    ~Sensing() {}

    // -------------------------------------------------------------------------
    // 1. THE UPDATE LOOP (Call this ONCE inside your while loop)
    // -------------------------------------------------------------------------
    void update() {
        // A. Update Distance Sensors (Convert to Meters)
        for (int i = 0; i < 8; i++) {
            if (i < distanceSensors.size()) {
                double raw = distanceSensors[i]->getValue();
                // Convert Raw (0-4096) to Meters (0.3m-0.0m) based on your Linear Lookup
                // Formula: Dist = MaxRange * (1 - (Raw / 4096))
                double meters = 0.185 * (1.0 - (raw / 4096.0));
                
                // Simple Filter: 60% new data, 40% old data to smooth jitter
                cached_dist_meters[i] = (0.6 * meters) + (0.4 * cached_dist_meters[i]);
            }
        }

        // B. Update Encoders & Calculate Yaw (ODOMETRY)
        if (leftEncoder && rightEncoder) {
            // 1. Get latest values into cache
            cached_enc_left = leftEncoder->getValue();
            cached_enc_right = rightEncoder->getValue();

            // 2. Calculate difference from last tick
            double diffL = cached_enc_left - lastLeftVal;
            double diffR = cached_enc_right - lastRightVal;

            // 3. Convert radians to meters traveled
            double distL = diffL * WHEEL_RADIUS;
            double distR = diffR * WHEEL_RADIUS;

            // 4. Calculate Yaw Change (Standard Odometry Formula)
            // If Right moves more than Left, we turn Left (Positive Yaw)
            double deltaYaw = (distR - distL) / AXLE_LENGTH;

            // 5. Accumulate Yaw
            currentYaw += deltaYaw;

            // 6. Update History for next tick
            lastLeftVal = cached_enc_left;
            lastRightVal = cached_enc_right;

            // 7. Store result in public cache
            cached_yaw = currentYaw;
        }

        // // B. Update IMU (Yaw / Heading)
        // if (imu) {
        //     const double* rpy = imu->getRollPitchYaw();
        //     if (!std::isnan(rpy[2])) cached_yaw = rpy[2]; // Index 2 is usually Yaw
        // }

        // C. Update Encoders
        if (leftEncoder) cached_enc_left = leftEncoder->getValue();
        if (rightEncoder) cached_enc_right = rightEncoder->getValue();

        // D. Update Vision
        cached_floor_color = processVision();
    }

    // -------------------------------------------------------------------------
    // 2. WALL DETECTION LOGIC (Updated to use Meters)
    // -------------------------------------------------------------------------
    // We now ask: "Is the wall closer than X meters?" 
    // This is safer than raw thresholds.
    
    bool isWallAtFront() {
        // Check front two sensors. If Both are closer than 15cm (0.15m)
        return (getDistance(0) < 0.15) && (getDistance(7) < 0.15);
    }

    bool isWallAtBack() {
        return (getDistance(3) < 0.1) && (getDistance(4) < 0.1);
    }

    bool isWallAtLeft() {
        // Check Side Left (5) and Diagonal Left (6)
        return (getDistance(5) < 0.15);
    }

    bool isWallAtRight() {
        // Check Side Right (2) and Diagonal Right (1)
        return (getDistance(2) < 0.15);
    }

    // Generic helper
    bool isWallDetected(int index) {
        return getDistance(index) < 0.15; // Threshold: 15cm
    }

    double getDistanceToFrontWall() {
        if (getDistance(0) == 0.185 || getDistance(7) == 0.185) return 0.185;
        return ((getDistance(0) + getDistance(7)) / 2);
    }

    double getDistanceToBackWall() {
        if (getDistance(3) == 0.185 || getDistance(4) == 0.185) return 0.185;
        return ((getDistance(3) + getDistance(4)) / 2);
    }

    double getDistanceToLeftWall() {
        return getDistance(5);
    }
    double getDistanceToRightWall() {
        return getDistance(2);
    }

    // -------------------------------------------------------------------------
    // 3. GETTERS (For your Math/Navigator files)
    // -------------------------------------------------------------------------
    double getDistance(int index) {
        if (index < 0 || index >= 8) return 99.0;
        return cached_dist_meters[index];
    }

    double getYaw() { return cached_yaw; }
    double getLeftEncoder() { return cached_enc_left; }
    double getRightEncoder() { return cached_enc_right; }
    FloorColor getFloorColor() { return cached_floor_color; }
    
    void setLEDs(bool on) {
        for(auto* l : leds) l->set(on ? 1 : 0);
    }

private:
    // Device Pointers
    webots::Robot *robot;
    int timeStep;
    std::vector<webots::DistanceSensor *> distanceSensors;
    std::vector<webots::LightSensor *> lightSensors;
    std::vector<webots::LED *> leds;
    webots::Accelerometer *accelerometer;
    webots::Camera *camera;
    webots::InertialUnit *imu;         // NEW
    webots::PositionSensor *leftEncoder; // NEW
    webots::PositionSensor *rightEncoder;// NEW

    // Cache Variables (The "Snapshot")
    double cached_dist_meters[8]; 
    double cached_yaw;
    double cached_enc_left;
    double cached_enc_right;
    FloorColor cached_floor_color;
    // E-puck Physical Constants
    const double WHEEL_RADIUS = 0.020; // Meters (20.5mm)
    const double AXLE_LENGTH = 0.052;   // Meters (52mm distance between wheels)

    // Odometry State Variables
    double currentYaw;
    double lastLeftVal;
    double lastRightVal;

    // Helper: Process Vision
    FloorColor processVision() {
        if (!camera) return COLOR_NONE;
        const unsigned char* image = camera->getImage();
        int w = camera->getWidth();
        int h = camera->getHeight();
        
        // Check center pixels
        int r = 0, g = 0, b = 0, count = 0;
        int startX = (w/2) - 2; 
        int startY = (h/2) - 2;

        for (int x = startX; x < startX + 5; x++) {
            for (int y = startY; y < startY + 5; y++) {
                // Bounds check
                if (x >=0 && x < w && y >= 0 && y < h) {
                    r += webots::Camera::imageGetRed(image, w, x, y);
                    g += webots::Camera::imageGetGreen(image, w, x, y);
                    b += webots::Camera::imageGetBlue(image, w, x, y);
                    count++;
                }
            }
        }
        if (count == 0) return COLOR_NONE;
        r /= count; g /= count; b /= count;

        if (g > r + 40 && g > b + 40) return COLOR_GREEN;
        return COLOR_NONE;
    }

    void initSensors() {
        // 1. Distance Sensors
        std::string psNames[8] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
        for (int i = 0; i < 8; i++) {
            webots::DistanceSensor *ds = robot->getDistanceSensor(psNames[i]);
            if (ds) {
                ds->enable(timeStep);
                distanceSensors.push_back(ds);
                cached_dist_meters[i] = 0.0; // Init cache
            }
        }

        // 2. Light Sensors
        std::string lsNames[8] = {"ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7"};
        for (int i = 0; i < 8; i++) {
            webots::LightSensor *ls = robot->getLightSensor(lsNames[i]);
            if (ls) {
                ls->enable(timeStep);
                lightSensors.push_back(ls);
            }
        }

        // 3. Accelerometer
        accelerometer = robot->getAccelerometer("accelerometer");
        if (accelerometer) accelerometer->enable(timeStep);

        // 4. Camera
        camera = robot->getCamera("camera");
        if (camera) camera->enable(timeStep);

        // 5. IMU (Gyro) - CRITICAL FOR YOU
        // imu = robot->getInertialUnit("inertial unit"); // Default e-puck name
        // if (imu) imu->enable(timeStep);
        // if (imu) { std::cout << "imu" << std::endl;}
        // Gyro (IMU not available)
        

        // 6. Encoders - CRITICAL FOR YOU
        leftEncoder = robot->getPositionSensor("left wheel sensor");
        rightEncoder = robot->getPositionSensor("right wheel sensor");
        if (leftEncoder) leftEncoder->enable(timeStep);
        if (rightEncoder) rightEncoder->enable(timeStep);
        
        // 7. LEDs
        std::string lNames[10] = {"led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"};
        for(int i=0; i<10; i++) {
            webots::LED* l = robot->getLED(lNames[i]);
            if(l) leds.push_back(l);
        }

        std::cout << "Sensors Initialized: IMU, Encoders, Distance, Vision." << std::endl;
    }
};
} // namespace Sensor

#endif // SENSING_H