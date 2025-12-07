
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <webots/Supervisor.hpp>


#include "motion_control.h"
#include "sensing.h"

using namespace webots;
using namespace Motion;
using namespace Sensor;

int main(int argc, char **argv) {
  // 1. Initialize Robot (Standard Supervisor)
  Supervisor *robot = new Supervisor();
  int timeStep = (int)robot->getBasicTimeStep();

  // 2. Initialize Modules
  Sensing *sensing = new Sensing(robot, timeStep);
  MotionController *motion = new MotionController(robot, sensing);

  // 3. Initial Stabilization
  for (int i = 0; i < 20; i++) {
    robot->step(timeStep);
    sensing->update();
  }

  // 4. Calibrate Gyro
  std::cout << "Calibrating Gyro..." << std::endl;
  sensing->calibrateGyro(50);

  // OVERWRITE YAW: Enforce North Heading to fix straight-line crash regression.
  sensing->overwriteYaw(1.5709);

  std::cout << ">>> SOLVER STARTED: Standard Drive Profile <<<" << std::endl;

  // 5. Execute Command
  motion->moveForward(1.2); // Move 1.2 Meters

  // 6. Main Control Loop
  while (robot->step(timeStep) != -1) {
    sensing->update();

    // Update Motion Controller with Delta Time
    double dt = timeStep / 1000.0;
    motion->update(dt);

    // Check Completion
    if (!motion->isBusy()) {
      std::cout << ">>> GOAL REACHED <<<" << std::endl;
      break; // Exit loop
    }
  }

  // 7. Cleanup
  delete motion;
  delete sensing;
  delete robot;
  return 0;
}