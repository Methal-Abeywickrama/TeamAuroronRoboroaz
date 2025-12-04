// File: movement_controller.cpp
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream> // For printing to console

// Standard namespace for Webots classes
using namespace webots;

// Constants for e-puck math
// Wheel radius = 20.5mm, Axle length = 52mm
const double MAX_SPEED = 6.28; 
const double TARGET_DISTANCE_IN_METERS = 0.5; // 2 squares * 0.25m
const double WHEEL_RADIUS = 0.0205;

// Formula: Arc Length = Radius * Theta -> Theta = Distance / Radius
// 0.5 / 0.0205 = ~24.39 radians
const double TARGET_RADS = TARGET_DISTANCE_IN_METERS / WHEEL_RADIUS;

int main(int argc, char **argv) {
  // 1. Create the Robot instance.
  Robot *robot = new Robot();

  // 2. Get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // 3. Initialize Motors
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");

  // CRITICAL: Set motors to "Velocity Mode" by setting position to Infinity
  // Otherwise, they will rotate to angle "0" and lock there.
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);

  // Start with 0 velocity
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  // 4. Initialize & Enable Position Sensors (Encoders)
  PositionSensor *leftSensor = robot->getPositionSensor("left wheel sensor");
  PositionSensor *rightSensor = robot->getPositionSensor("right wheel sensor");
  
  // You MUST enable sensors with the timestep, or they will return "NAN"
  leftSensor->enable(timeStep);
  rightSensor->enable(timeStep);

  // Main loop
  while (robot->step(timeStep) != -1) {
    // A. Read Sensor Data
    double currentPos = leftSensor->getValue();

    // Debugging: Print current position to the console
    std::cout << "Current Position: " << currentPos 
              << " / Target: " << TARGET_RADS << std::endl;

    // B. Logic: Are we there yet?
    if (currentPos < TARGET_RADS) {
        // Move Forward
        leftMotor->setVelocity(MAX_SPEED);
        rightMotor->setVelocity(MAX_SPEED);
    } else {
        // Stop
        leftMotor->setVelocity(0.0);
        rightMotor->setVelocity(0.0);
        
        // Optional: Exit loop to finish program
        // break; 
    }
  };

  // Cleanup
  delete robot;
  return 0;
}