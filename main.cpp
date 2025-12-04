#include <iostream>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>


#include "math_utils.h"
#include "motion_control.h"
#include "navigator.h"
#include "sensing.h"


using namespace webots;
using namespace Motion;
using namespace Sensor;
using namespace Navigation;
using namespace MathUtil;

int main(int argc, char **argv) {
  // Create the Robot instance.
  Robot *robot = new Robot();

  // Get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // Initialize subsystems
  MotionControl motion(robot);
  Sensing sensing(robot, timeStep);
  Navigator navigator;
  // MathUtils is static

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  }

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
