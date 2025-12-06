
#include <iostream>
#include <webots/Robot.hpp>

#include "motion_control.h"
#include "sensing.h"

using namespace webots;
using namespace Motion;
using namespace Sensor;

int main(int argc, char **argv) {
  // 1. Initialize Robot & TimeStep
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  std::cout << ">>> E-puck Calibration Harness Started <<<" << std::endl;

  // 2. Initialize Modules
  // We pass the 'robot' pointer to both modules so they can access devices.
  Sensing *sensing = new Sensing(robot, timeStep);
  MotionController *motion = new MotionController(robot, sensing);

  // 3. Settling Phase
  // Give physics a moment to stabilize before moving (prevents initial
  // weirdness)
  std::cout << "Status: Waiting for physics to settle..." << std::endl;
  for (int i = 0; i < 20; i++) {
    robot->step(timeStep);
    sensing->update();
  }
  robot->getMotor("left wheel motor")->setVelocity(0.0);
  robot->getMotor("right wheel motor")->setVelocity(0.0);
  // 3b. Calibrate Gyro (MUST be done while stationary)
  // Increased to 100 samples for better bias estimation
  sensing->calibrateGyro(100);

  // 4. Start Calibration Move
  // We command the robot to move exactly 1.25 meter forward.
  double commandDist = 1.25;
  // motion->moveForward(commandDist); // Commented out for Rotation Tuning

  // 5. Execution Loop
  std::cout << "Status: Executing Move..." << std::endl;

  bool hasTurned = false; // Simple flag to chain commands in this basic harness

  while (robot->step(timeStep) != -1) {
    // A. Update Sensors (Reads Encoders/IMU)
    sensing->update();

    // B. Update Motion (PID calculation)
    double dt = timeStep / 1000.0;
    motion->update(dt);

    // C. Exit/Chain when done
    if (!motion->isBusy()) {
      if (!hasTurned) {
        std::cout << "Status: Forward Complete. Turning Left..." << std::endl;
        motion->turnLeft(1); // 90 Degrees
        hasTurned = true;
      } else {
        std::cout << "Status: All Commands Complete!" << std::endl;
        break;
      }
    }
  }

  // 6. Post-Run Diagnostics
  std::cout << "------------------------------------------------" << std::endl;
  std::cout << ">>> CALIBRATION INSTRUCTIONS <<<" << std::endl;
  std::cout << "1. Measure the REAL distance the robot traveled in Webots."
            << std::endl;
  std::cout << "   (Use the Ruler tool or Translation field in the 3D view)"
            << std::endl;
  std::cout << "2. Expected (Internal) Distance: " << commandDist << " m"
            << std::endl;
  std::cout << "3. Calculate New Radius:" << std::endl;
  std::cout << "   NewRadius = (RealDistance / " << commandDist
            << ") * OldRadius" << std::endl;
  std::cout << "   Current OldRadius in code: 0.0205 (Default)" << std::endl;
  std::cout << "------------------------------------------------" << std::endl;

  // Cleanup
  delete motion;
  delete sensing;
  delete robot;
  return 0;
}
