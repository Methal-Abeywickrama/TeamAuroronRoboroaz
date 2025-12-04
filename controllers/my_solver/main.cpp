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
  // 1. Initialize the Robot
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  // 2. Proof of Life (Check your Terminal for this!)
  std::cout << ">>> SUCCESS: Main is running and connected! <<<" << std::endl;

  // 3. Get Motors
  Motor *left = robot->getMotor("left wheel motor");
  Motor *right = robot->getMotor("right wheel motor");

  // 4. Spin in place (Visual check)
  if (left && right) {
    left->setPosition(INFINITY);
    right->setPosition(INFINITY);

    left->setVelocity(5.0);
    right->setVelocity(5.0); // Negative speed makes it spin
  }

  // 5. Keep running
  while (robot->step(timeStep) != -1) {
    // Robot spins as long as this loop runs
  }

  delete robot;
  return 0;
}
