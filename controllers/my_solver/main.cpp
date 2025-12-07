
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <webots/Supervisor.hpp>

// #include "motion_control.h"
#include "motion_control.h"
#include "sensing.h"

using namespace webots;
using namespace Motion;
using namespace Sensor;

int main(int argc, char **argv) {
  // 1. Initialize Robot
  Supervisor *robot = new Supervisor();
  int timeStep = (int)robot->getBasicTimeStep();

  // 2. Initialize Modules
  Sensing *sensing = new Sensing(robot, timeStep);
  MovingController *motion = new MovingController(robot, sensing);

  // 3. Initial Stabilization
  for (int i = 0; i < 20; i++) {
    robot->step(timeStep);
    sensing->update();
  }

  // 4. Calibrate Gyro
  std::cout << "Calibrating Gyro..." << std::endl;
  sensing->calibrateGyro(50);

  std::cout << ">>> MAZE SOLVER STARTED <<<" << std::endl;

  // 5. Command Sequence
  enum Step { FORWARD_5, TURN_LEFT, FORWARD_1, DONE };
  Step currentStep = FORWARD_5;

  // Start first command
  motion->moveForward(5); // 5 tiles = 1.25m

  // 6. Main Control Loop
  while (robot->step(timeStep) != -1) {
    sensing->update();

    double dt = timeStep / 1000.0;
    motion->update(dt);

    // Check if current command finished
    if (!motion->isBusy()) {
      switch (currentStep) {
      case FORWARD_5:
        std::cout << ">>> 5 TILES DONE, TURNING LEFT <<<" << std::endl;
        motion->turnLeft();
        currentStep = TURN_LEFT;
        break;
      case TURN_LEFT:
        std::cout << ">>> TURN DONE, MOVING 1 TILE <<<" << std::endl;
        motion->moveForward(1); // 1 tile = 0.25m
        currentStep = FORWARD_1;
        break;
      case FORWARD_1:
        std::cout << ">>> ALL COMMANDS COMPLETE <<<" << std::endl;
        currentStep = DONE;
        break;
      case DONE:
        // Exit loop
        break;
      }
      if (currentStep == DONE)
        break;
    }
  }

  // 7. Cleanup
  delete motion;
  delete sensing;
  delete robot;
  return 0;
}
