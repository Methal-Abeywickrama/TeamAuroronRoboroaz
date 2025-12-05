#include <iomanip>
#include <iostream>
#include <webots/Robot.hpp>

#include "motion_control.h"
// #include "sensing.h" // Future integration

using namespace webots;
using namespace Motion;

int main(int argc, char **argv) {
  // 1. Initialize the Robot (Required for Webots controllers)
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  std::cout << ">>> PID Control Test Harness Started <<<" << std::endl;

  // 2. Configure PID
  PIDConfig pidConfig;
  pidConfig.Kp = 2.0;
  pidConfig.Ki = 0.1;
  pidConfig.Kd = 0.5;
  pidConfig.max_output = 6.28; // Max speed (rad/s)
  pidConfig.min_output = -6.28;
  pidConfig.max_integral = 2.0;            // Anti-windup limit
  pidConfig.derivative_filter_alpha = 0.1; // Smoothing factor

  PID pid(pidConfig);

  // 3. Simulation Parameters
  double target_position = 10.0;
  double current_position = 0.0;
  double dt = timeStep / 1000.0; // Convert ms to seconds

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "Target: " << target_position << std::endl;
  std::cout << "Step | Current | Error | Output | P-Term | I-Term | D-Term"
            << std::endl;
  std::cout << "-------------------------------------------------------------"
            << std::endl;

  // 4. Run Test Loop (Simulate 50 steps)
  for (int i = 0; i < 100; i++) {
    // Calculate PID output
    PIDResult result = pid.calculate(target_position, current_position, dt);

    // Print Status
    std::cout << std::setw(4) << i << " | " << std::setw(7) << current_position
              << " | " << std::setw(5) << (target_position - current_position)
              << " | " << std::setw(6) << result.output << " | " << std::setw(6)
              << result.p_term << " | " << std::setw(6) << result.i_term
              << " | " << std::setw(6) << result.d_term << std::endl;

    // Simulate Physics (Simple integration: position += velocity * dt)
    // In a real robot, this would be the motor moving the robot
    current_position += result.output * dt;

    // Step the Webots simulation (keep connection alive)
    if (robot->step(timeStep) == -1)
      break;
  }

  std::cout << ">>> Test Complete <<<" << std::endl;

  delete robot;
  return 0;
}
