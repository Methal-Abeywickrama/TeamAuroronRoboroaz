#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <webots/Motor.hpp>
#include <webots/Robot.hpp>


namespace Motion {

class MotionControl {
public:
  MotionControl(webots::Robot *robot) {
    this->robot = robot;
    // Initialize motors
    leftMotor = robot->getMotor("left wheel motor");
    rightMotor = robot->getMotor("right wheel motor");

    if (leftMotor) {
      leftMotor->setPosition(INFINITY);
      leftMotor->setVelocity(0.0);
    }
    if (rightMotor) {
      rightMotor->setPosition(INFINITY);
      rightMotor->setVelocity(0.0);
    }
  }

  ~MotionControl() {}

  void setSpeed(double left, double right) {
    if (leftMotor)
      leftMotor->setVelocity(left);
    if (rightMotor)
      rightMotor->setVelocity(right);
  }

  void stop() { setSpeed(0.0, 0.0); }

private:
  webots::Robot *robot;
  webots::Motor *leftMotor;
  webots::Motor *rightMotor;
};
} // namespace Motion

#endif // MOTION_CONTROL_H
