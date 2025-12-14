# Sensing Module (`Sensing.h`) — README

This module provides hardware abstraction for the e-puck robot. It initializes sensors, processes raw data into useful units (meters, radians), and manages the robot’s perception state.

**Key Feature:**  
Supports two side-by-side methods for tracking rotation (Yaw):
- Odometry (wheel-based)
- Gyroscope-based

---

## Table of Contents

- [Quick Start](#quick-start)
- [Rotation / Yaw](#rotation--yaw)
- [Distance Sensors](#distance-sensors)
- [Other Sensors](#other-sensors)
- [Math & Calibration](#math--calibration)

---

## Quick Start

### Initialization

Include the header and create an instance in your `main.cpp`:

```cpp
#include "Sensing.h"

// ... inside main()
Sensor::Sensing *sensing = new Sensor::Sensing(robot, timeStep);

## 2. Rotation / Yaw Functions
We currently track two separate yaw values for testing purposes. Both values are in Radians.

0.0 = The angle when the program started (or last reset).

Positive (+) = Turned Left (Counter-Clockwise).

Negative (-) = Turned Right (Clockwise).

## 3. Distance Sensor Functions
All distance functions return values in Meters.

Range: ~0.02m (Very close) to 2.0m (Infinity/Far).

Wall Threshold: Generally, < 0.15 meters means a wall is close.

High-Level Checks (Boolean)

Use these for simple if statements.

bool isWallAtFront(): Checks if both front sensors see a wall (< 15cm).

bool isWallAtLeft(): Checks the left side sensor.

bool isWallAtRight(): Checks the right side sensor.

bool isWallAtBack(): Checks the rear sensors.

Precise Measurements (Double)

Use these for PID control or logic.

double getDistanceToFrontWall(): Returns average distance of the two front sensors.

double getDistanceToLeftWall(): Returns distance to the left wall.

double getDistanceToRightWall(): Returns distance to the right wall.

Low-Level Access

double getDistance(int index): Get specific sensor value (0-7).

0 & 7: Front

2: Right

5: Left

3 & 4: Back

4. Other Sensors
Vision

FloorColor getFloorColor(): Returns detected color under the robot.

Values: COLOR_NONE, COLOR_RED, COLOR_GREEN, COLOR_CHECKERBOARD.

Encoders

double getLeftEncoder(): Total radians the left wheel has spun.

double getRightEncoder(): Total radians the right wheel has spun.

5. Math & Calibration Notes
Odometry Calibration

If getYawOdometry() consistently undershoots or overshoots a 360° turn:

Adjust AXLE_LENGTH in Sensing.h (Default: 0.052).

Adjust WHEEL_RADIUS in Sensing.h (Default: 0.0205).

Distance Calibration

We use a Power Law formula to convert raw IR values to meters:

meters=0.185×(100/raw)^-0.70

 
If distances are wrong, tweak the -0.70 exponent.