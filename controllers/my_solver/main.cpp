
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <webots/Supervisor.hpp>

// #include "moving_control.h"
#include "motion_control.h"
#include "sensing.h"

using namespace webots;
using namespace Motion;
using namespace Sensor;

// ============================================================================
// COMMAND TYPES - Use these to build your command sequence
// ============================================================================
enum CommandType {
  FORWARD,   // Move forward N tiles
  TURN_LEFT, // Turn 90 degrees left
  TURN_RIGHT // Turn 90 degrees right
};

struct Command {
  CommandType type;
  double value; // For FORWARD: number of tiles. For turns: ignored (use 0)
};

// ============================================================================
// >>> EDIT YOUR COMMAND SEQUENCE HERE <<<
// ============================================================================
const std::vector<Command> COMMANDS = {
    //{FORWARD, 5},   // Go forward 5 tiles
    {TURN_LEFT, 0}, // Turn left 90 degrees
    {TURN_LEFT, 0},
    {TURN_LEFT, 0},
    {TURN_LEFT, 0}, // Add more commands here as needed, e.g.:
                    // {FORWARD, 3},
                    // {TURN_RIGHT, 0},
                    // {FORWARD, 2},
};

// ============================================================================
// MAIN FUNCTION
// ============================================================================
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
  std::cout << "Total commands to execute: " << COMMANDS.size() << std::endl;

  // 5. Execute Command Sequence
  size_t cmdIndex = 0;
  bool needNextCommand = true;

  while (robot->step(timeStep) != -1) {
    sensing->update();
    double dt = timeStep / 1000.0;
    motion->update(dt);

    // Start next command when ready
    if (needNextCommand && !motion->isBusy()) {
      if (cmdIndex >= COMMANDS.size()) {
        std::cout << ">>> ALL COMMANDS COMPLETE <<<" << std::endl;
        break;
      }

      const Command &cmd = COMMANDS[cmdIndex];
      std::cout << "[CMD " << (cmdIndex + 1) << "/" << COMMANDS.size() << "] ";

      switch (cmd.type) {
      case FORWARD:
        std::cout << "FORWARD " << cmd.value << " tiles" << std::endl;
        motion->moveForward(cmd.value);
        break;
      case TURN_LEFT:
        std::cout << "TURN LEFT" << std::endl;
        motion->turnLeft();
        break;
      case TURN_RIGHT:
        std::cout << "TURN RIGHT" << std::endl;
        motion->turnRight();
        break;
      }

      cmdIndex++;
      needNextCommand = false;
    }

    // Check if command finished
    if (!motion->isBusy()) {
      needNextCommand = true;
    }
  }

  // 6. Cleanup
  delete motion;
  delete sensing;
  delete robot;
  return 0;
}
