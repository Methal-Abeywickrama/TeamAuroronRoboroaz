// DFS Maze Exploration - Main Controller
// Uses explorer.h for DFS, navigator.h for data, motion_control.h for movement

#include "explorer.h"
#include "motion_control.h"
#include "navigator.h"
#include "sensing.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <webots/Robot.hpp>

// Tee buffer: writes to both console and file
class TeeBuffer : public std::streambuf {
public:
  TeeBuffer(std::streambuf *console, std::streambuf *file)
      : console_(console), file_(file) {}

protected:
  int overflow(int c) override {
    if (c == EOF)
      return !EOF;
    if (console_->sputc(c) == EOF)
      return EOF;
    if (file_->sputc(c) == EOF)
      return EOF;
    return c;
  }

  int sync() override {
    console_->pubsync();
    file_->pubsync();
    return 0;
  }

private:
  std::streambuf *console_;
  std::streambuf *file_;
};

int main() {
  // Open debug file FIRST (before any cout)
  std::ofstream debugFile("debug_report.txt");

  // Save original cout buffer and set up tee
  std::streambuf *originalCoutBuf = std::cout.rdbuf();
  TeeBuffer teeBuf(originalCoutBuf, debugFile.rdbuf());
  std::cout.rdbuf(&teeBuf);

  webots::Robot robot;
  int timeStep = (int)robot.getBasicTimeStep();
  double dt = timeStep / 1000.0;

  // Initialize components
  Sensor::Sensing sensing(&robot, timeStep);
  Motion::MovingController motion(&robot, &sensing);
  Navigation::Navigator navigator;

  // Initialize logger
  navigator.initLogger("exploration_log.txt");

  // Create explorer
  Exploration::Explorer explorer(&robot, &sensing, &motion, &navigator);

  // Calibrate gyro
  std::cout << "Calibrating gyro..." << std::endl;
  sensing.calibrateGyro(20);

  std::cout << "\n========================================" << std::endl;
  std::cout << "     DFS MAZE EXPLORATION" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "Grid: 12x12" << std::endl;
  std::cout << "Start: (0,0) bottom-right, facing NORTH" << std::endl;
  std::cout << "----------------------------------------\n" << std::endl;

  // Start exploration
  explorer.start();

  // Main loop
  double lastStatusTime = 0;
  while (robot.step(timeStep) != -1) {
    sensing.update();

    // Run explorer
    bool complete = explorer.update(dt);

    // Print status every 2 seconds (TeeBuffer writes to both console and file)
    double now = robot.getTime();
    if (now - lastStatusTime > 2.0) {
      std::cout << std::fixed << std::setprecision(1);
      std::cout << "[" << now << "s] Cell(" << explorer.getCurrentX() << ","
                << explorer.getCurrentY() << ") " << explorer.getHeadingStr()
                << " - " << explorer.getStateStr()
                << " FW:" << std::setprecision(3)
                << sensing.getDistanceToFrontWall() << "m" << std::endl;
      lastStatusTime = now;
    }

    if (complete) {
      // Check what type of completion
      if (explorer.isDestinationReached()) {
        std::cout << "\n========================================" << std::endl;
        std::cout << "     MISSION COMPLETE!" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Robot has reached the destination." << std::endl;
      } else {
        std::cout << "\n========================================" << std::endl;
        std::cout << "     EXPLORATION/NAVIGATION COMPLETE!" << std::endl;
        std::cout << "========================================" << std::endl;
      }

      // Print final position
      std::cout << "Final position: (" << explorer.getCurrentX() << ","
                << explorer.getCurrentY() << ")" << std::endl;

      // Print map
      std::cout << "\nExplored Map:" << std::endl;
      navigator.debugPrintMap();

      // Print checkpoints
      auto checkpoints = navigator.getMap().getCheckpoints();
      std::cout << "\nCheckpoints found: " << checkpoints.size() << std::endl;
      for (const auto &cp : checkpoints) {
        std::cout << "  (" << cp.x << "," << cp.y << ")" << std::endl;
      }

      // Destination
      auto dest = navigator.getDestination();
      std::cout << "\nCalculated Destination: (" << dest.x << "," << dest.y
                << ")" << std::endl;

      navigator.flushLog();

      // Idle loop
      std::cout << "\nRobot idle. Simulation continues..." << std::endl;
      while (robot.step(timeStep) != -1) {
        sensing.update();
      }
      break;
    }
  }

  return 0;
}
