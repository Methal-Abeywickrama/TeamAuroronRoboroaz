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
#include <webots/Camera.hpp>
#include <webots/PositionSensor.hpp> 
#include <webots/Gyro.hpp> // <--- Added Gyro
#include <webots/LED.hpp>            

namespace Sensor {

enum FloorColor { COLOR_NONE, COLOR_RED, COLOR_GREEN, COLOR_CHECKERBOARD };

class Sensing {
public:
    Sensing(webots::Robot *robot, int timeStep) {
        this->robot = robot;
        this->timeStep = timeStep;
        
        // Pre-calculate dt in seconds for Gyro math (ms -> seconds)
        this->dt_seconds = timeStep / 1000.0;

        // Init logic variables
        yaw_odometry = 0.0;
        yaw_gyro = 0.0;
        
        lastLeftVal = 0.0;
        lastRightVal = 0.0;
        
        initSensors();
    }

    ~Sensing() {}

    // -------------------------------------------------------------------------
    // 1. UPDATE LOOP
    // -------------------------------------------------------------------------
    void update() {
        // A. METHOD 1: Odometry (Wheel Encoders)
        if (leftEncoder && rightEncoder) {
            cached_enc_left = leftEncoder->getValue();
            cached_enc_right = rightEncoder->getValue();

            double diffL = cached_enc_left - lastLeftVal;
            double diffR = cached_enc_right - lastRightVal;

            double distL = diffL * WHEEL_RADIUS;
            double distR = diffR * WHEEL_RADIUS;

            // Calculate Angle Change
            double deltaYaw = (distR - distL) / AXLE_LENGTH;
            yaw_odometry += deltaYaw;

            // Update History
            lastLeftVal = cached_enc_left;
            lastRightVal = cached_enc_right;
        }

        // B. METHOD 2: Gyroscope (Integration)
        if (gyro) {
            const double *values = gyro->getValues();
            // Index 0=X, 1=Y, 2=Z (Yaw axis for E-puck)
            if (!std::isnan(values[2])) {
                double angularVelocity = values[2]; // rad/s
                
                // MATH: Angle = Rate * Time
                double deltaGyro = angularVelocity * dt_seconds;
                yaw_gyro += deltaGyro; 
            }
        }

        // C. Update Distance Sensors (Using Power Law)
        for (int i = 0; i < 8; i++) {
            if (i < distanceSensors.size()) {
                double raw = distanceSensors[i]->getValue();
                
                double meters;
                // If raw is tiny, it's far away
                if (raw < 80.0) {
                    meters = 2.0; 
                } else {
                    // Power Law for better accuracy
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
    // 2. UTILITIES
    // -------------------------------------------------------------------------
    
    // Reset BOTH Yaws to 0
    void resetYaw() {
        yaw_odometry = 0.0;
        yaw_gyro = 0.0;
        std::cout << "Both Yaw values reset to 0." << std::endl;
    }

    // -------------------------------------------------------------------------
    // 3. GETTERS
    // -------------------------------------------------------------------------
    
    // METHOD 1 GETTER
    double getYawOdometry() { return yaw_odometry; }
    
    // METHOD 2 GETTER
    double getYawGyro() { return yaw_gyro; }

    // Distance Getters
    double getDistance(int index) {
        if (index < 0 || index >= 8) return 99.0;
        return cached_dist_meters[index];
    }
    
    // Encoder Getters
    double getLeftEncoder() { return cached_enc_left; }
    double getRightEncoder() { return cached_enc_right; }
    FloorColor getFloorColor() { return cached_floor_color; }
    
    // Wall Helpers
    bool isWallAtFront() { return (getDistance(0) < 0.15) && (getDistance(7) < 0.15); }
    bool isWallAtBack() { return (getDistance(3) < 0.1) && (getDistance(4) < 0.1); }
    bool isWallAtLeft() { return (getDistance(5) < 0.15); }
    bool isWallAtRight() { return (getDistance(2) < 0.15); }
    
    double getDistanceToFrontWall() {
         double d1 = getDistance(0);
         double d2 = getDistance(7);
         if(d1 > 1.0 || d2 > 1.0) return 2.0; 
         return (d1 + d2) / 2.0;
    }
    
    double getDistanceToLeftWall() { return getDistance(5); }
    double getDistanceToRightWall() { return getDistance(2); }

private:
    // Device Pointers
    webots::Robot *robot;
    int timeStep;
    double dt_seconds; // Needed for Gyro math
    
    std::vector<webots::DistanceSensor *> distanceSensors;
    std::vector<webots::LightSensor *> lightSensors;
    std::vector<webots::LED *> leds;
    webots::Camera *camera;
    webots::PositionSensor *leftEncoder; 
    webots::PositionSensor *rightEncoder;
    webots::Gyro *gyro; // <--- The Gyro Pointer

    // Cache Variables 
    double cached_dist_meters[8]; 
    double cached_enc_left;
    double cached_enc_right;
    FloorColor cached_floor_color;
    
    // E-puck Physical Constants
    const double WHEEL_RADIUS = 0.0205; 
    const double AXLE_LENGTH = 0.052;   

    // YAW VARIABLES
    double yaw_odometry;
    double yaw_gyro;
    
    // Odometry History
    double lastLeftVal;
    double lastRightVal;

    // Helper: Process Vision
    FloorColor processVision() {
        if (!camera) return COLOR_NONE;
        const unsigned char* image = camera->getImage();
        int w = camera->getWidth();
        int h = camera->getHeight();
        
        int r = 0, g = 0, b = 0, count = 0;
        int startX = (w/2) - 2; 
        int startY = (h/2) - 2;

        for (int x = startX; x < startX + 5; x++) {
            for (int y = startY; y < startY + 5; y++) {
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
                cached_dist_meters[i] = 0.0; 
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

        // 3. Camera
        camera = robot->getCamera("camera");
        if (camera) camera->enable(timeStep);

        // 4. Encoders
        leftEncoder = robot->getPositionSensor("left wheel sensor");
        rightEncoder = robot->getPositionSensor("right wheel sensor");
        if (leftEncoder) leftEncoder->enable(timeStep);
        if (rightEncoder) rightEncoder->enable(timeStep);

        // 5. Gyro (Try to find it)
        gyro = robot->getGyro("gyro"); 
        if (gyro) {
            gyro->enable(timeStep);
            std::cout << "Sensing: Gyro found and enabled." << std::endl;
        } else {
            std::cout << "Sensing: WARNING - Gyro NOT found." << std::endl;
        }
        
        // 6. LEDs
        std::string lNames[10] = {"led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"};
        for(int i=0; i<10; i++) {
            webots::LED* l = robot->getLED(lNames[i]);
            if(l) leds.push_back(l);
        }
    }
};
} // namespace Sensor

#endif // SENSING_H