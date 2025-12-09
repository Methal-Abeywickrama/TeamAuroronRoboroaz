#include <iostream>
#include <webots/Supervisor.hpp>

#include "motion_control.h"
#include "sensing.h"
// #include "testingfunctions.h"  // No longer needed for moveForwardFast

using namespace webots;
using namespace Sensor;
using namespace Motion;

// ============================================================================
// MAIN FUNCTION - Test moveForwardFast (now in MotionController)
// ============================================================================
int main(int argc, char **argv) {
  // 1. Initialize Robot
  Supervisor *robot = new Supervisor();
  int timeStep = (int)robot->getBasicTimeStep();

  // 2. Initialize Sensing
  Sensing *sensing = new Sensing(robot, timeStep);

  // 3. Initial Stabilization
  std::cout << "Stabilizing..." << std::endl;
  for (int i = 0; i < 20; i++) {
    robot->step(timeStep);
    sensing->update();
  }

  // 4. Calibrate Gyro
  std::cout << "Calibrating Gyro..." << std::endl;
  sensing->calibrateGyro(50);

  // 5. Create MotionController
  MovingController motion(robot, sensing);

  // =========================================================================
  // TEST: 1-tile movement with wall correction (using MotionController)
  // Target: Robot center at 125mm from wall (sensor reads 90mm)
  // =========================================================================
  std::cout << "\n>>> 5-TILE TEST WITH WALLFAILSAFE & CORRECTION <<<"
            << std::endl;
  std::cout << "Moving 5 tiles. Wall correction logic runs after each tile."
            << std::endl;
  std::cout << "----------------------------------------" << std::endl;

  for (int i = 0; i < 5; i++) {
    std::cout << "\n=== MOVING TILE " << (i + 1) << "/5 ===" << std::endl;
    motion.moveForwardFast();

    // Check for wall and correct position if detected
    if (motion.hasWallInFront()) {
      std::cout << "Wall detected - correcting to 90mm..." << std::endl;
      motion.correctToWall(0.09000); // 90mm sensor = 125mm center
    } else {
      std::cout << "No wall in range" << std::endl;
    }
  }

  std::cout << "----------------------------------------" << std::endl;
  std::cout << ">>> TEST COMPLETE <<<" << std::endl;
  std::cout << "Check testing_debugs.txt for detailed log" << std::endl;

  // 6. Keep simulation running briefly to see final position
  for (int i = 0; i < 50; i++) {
    robot->step(timeStep);
  }

  // 7. Cleanup
  delete sensing;
  delete robot;
  return 0;
}
