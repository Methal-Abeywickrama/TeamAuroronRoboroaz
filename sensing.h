#ifndef SENSING_H
#define SENSING_H

#include <string>
#include <vector>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Robot.hpp>


namespace Sensor {

class Sensing {
public:
  Sensing(webots::Robot *robot, int timeStep) {
    this->robot = robot;
    this->timeStep = timeStep;
    initSensors();
  }

  ~Sensing() {}

  // Add methods to get sensor values here
  // double getDistanceSensorValue(int index);

private:
  webots::Robot *robot;
  int timeStep;

  // E-puck has 8 distance sensors usually named ps0-ps7
  std::vector<webots::DistanceSensor *> distanceSensors;
  // E-puck has 8 light sensors usually named ls0-ls7
  std::vector<webots::LightSensor *> lightSensors;

  webots::Accelerometer *accelerometer;
  webots::Camera *camera;

  void initSensors() {
    // Initialize Distance Sensors
    char psNames[8][4] = {"ps0", "ps1", "ps2", "ps3",
                          "ps4", "ps5", "ps6", "ps7"};

    for (int i = 0; i < 8; i++) {
      webots::DistanceSensor *ds = robot->getDistanceSensor(psNames[i]);
      if (ds) {
        ds->enable(timeStep);
        distanceSensors.push_back(ds);
      }
    }

    // Initialize Light Sensors
    char lsNames[8][4] = {"ls0", "ls1", "ls2", "ls3",
                          "ls4", "ls5", "ls6", "ls7"};

    for (int i = 0; i < 8; i++) {
      webots::LightSensor *ls = robot->getLightSensor(lsNames[i]);
      if (ls) {
        ls->enable(timeStep);
        lightSensors.push_back(ls);
      }
    }

    // Initialize Accelerometer
    accelerometer = robot->getAccelerometer("accelerometer");
    if (accelerometer) {
      accelerometer->enable(timeStep);
    }

    // Initialize Camera
    camera = robot->getCamera("camera");
    if (camera) {
      camera->enable(timeStep);
    }
  }
};
} // namespace Sensor

#endif // SENSING_H
