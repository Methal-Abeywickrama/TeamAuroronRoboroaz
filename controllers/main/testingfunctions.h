#ifndef TESTING_FUNCTIONS_H
#define TESTING_FUNCTIONS_H

#include "sensing.h"
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#define L_ERROR 0.01
#define CELL_SIZE 0.2

namespace Navigation {

// Use M_PI if available, otherwise define a constant (Better than #define)
#ifndef M_PI
constexpr double PI = 3.1415926535;
#else
constexpr double PI = M_PI;
#endif

// --- 1. Vector Struct ---
template <typename T> struct Vec2 {
  T x, y;
  Vec2() : x(0), y(0) {}
  Vec2(T x_val, T y_val) : x(x_val), y(y_val) {}

  Vec2 operator-(const Vec2 &other) const { return {x - other.x, y - other.y}; }
  bool operator!=(const Vec2 &other) const { return !(*this == other); }
  bool operator==(const Vec2 &other) const {
    return x == other.x && y == other.y;
  }
};

// --- 2. Odometry Class ---
class Odometry {
private:
  double radius;
  double axleLength;
  double x;
  double y;
  double theta;
  double cellSize_;

public:
  // Default constructor required for Map class member
  Odometry()
      : radius(0.0), axleLength(0.0), x(0.0), y(0.0), theta(0.0),
        cellSize_(0.25) {}

  Odometry(double r, double l, double s)
      : radius(r), axleLength(l), x(0.0), y(0.0), theta(0.0), cellSize_(s) {}

  void update(double left_rad, double right_rad, double dt) {
    // left and right encoders (change)
    double dist_left = radius * left_rad;
    double dist_right = radius * right_rad;

    double d = (dist_left + dist_right) / 2;
    // double w = (v_right - v_left) / axleLength;

    x += d * std::cos(theta) * dt;
    y += d * std::sin(theta) * dt;
    // theta += w * dt;

    if (theta > PI) {
      theta -= 2 * PI;
    }
    if (theta <= -PI) {
      theta += 2 * PI;
    }
  }

  Vec2<double> getPos() const { return {x, y}; }

  int getGrid(double z) const {
    int coord = static_cast<int>(std::round(z / cellSize_));
    return coord;
  }

  double getX() const { return x; }
  double getY() const { return y; }
  double getAngle() const { return theta; }
  void setAngle(double ang) { theta = ang; }
};

// --- 3. Map Class ---
class Map {
private:
  int column;
  int row;
  Vec2<int> origin;
  double cell_size;
  int X_check, Y_check;
  std::vector<int> map;
  std::vector<Vec2<int>> checkpoints;
  Odometry odem;

public:
  Map(int r, int c, Vec2<int> o, double size, const Odometry &Odem) {
    column = c;
    row = r;
    origin = o;
    cell_size = size;
    X_check = 0;
    Y_check = 0;
    map.resize(r * c, 0);
    odem = Odem;
  }

  int index(int x, int y) const { return (y * column + x); }

  void update_cell(bool data[4], int coord_x, int coord_y) {
    if (!isValid(coord_x, coord_y))
      return;
    int i = index(coord_x, coord_y);
    int cell_num = 1 * data[0] + 2 * data[1] + 4 * data[2] + 8 * data[3];

    if (!map[i]) {
      map[i] = cell_num;
    }
  }

  void head(bool north_rb, bool east_rb, bool south_rb, bool west_rb,
            int coord_x, int coord_y) {
    int i = index(coord_x, coord_y);
    if (map[i]) {
      return;
    }
    bool sensorData[4] = {west_rb, north_rb, east_rb, south_rb};
    bool updateData[4];
    int headI = static_cast<int>(std::round(odem.getAngle() / (PI / 2))) + 1;
    for (int k = 0; k < 4; k++) {
      updateData[k] = sensorData[headI];
      headI += 1;
      headI = headI % 4;
    }
    update_cell(updateData, odem.getX(), odem.getY());
  }

  void updateMap(bool north_rb, bool east_rb, bool south_rb, bool west_rb) {
    double posX = odem.getX();
    double posY = odem.getY();
    if ((fmod(posX, CELL_SIZE) < L_ERROR) &&
        (fmod(posY, CELL_SIZE) < L_ERROR)) {
      int gridX = odem.getGrid(posX);
      int gridY = odem.getGrid(posY);
      head(north_rb, east_rb, south_rb, west_rb, gridX, gridY);
    }
    return;
  }

  void printMap() const {
    for (int i = 0; i < row; i++) {
      for (int j = 0; j < column; j++) {
        int k = index(j, i);
        std::cout << map[k] << ' ';
      }
      std::cout << std::endl;
    }
  }

  Vec2<int> getDim() const { return {row, column}; }

  int getValue(int x, int y) const {
    if (!isValid(x, y))
      return 0;
    int ind = index(x, y);
    return map[ind];
  }

  bool isValid(int x, int y) const {
    return (y >= 0 && y < row && x >= 0 && x < column);
  }

  void updateCheckpointCount(int x, int y) {
    Vec2<int> point = {x, y};
    for (const auto &visited : checkpoints) {
      if (visited == point) {
        // We have been here before!
        return;
      }
    }
    checkpoints.push_back(point);
    X_check += x;
    Y_check += y;
    return;
  }

  Vec2<int> getDest() const {
    int dest_x = X_check % 12;
    int dest_y = Y_check % 12;
    return {dest_x, dest_y};
  }
};

// --- 4. Helper Structures & Functions ---

struct cell {
  int parent_i, parent_j;
  double f, g, h;
};

inline bool EEquivalence(double a, double b, double error) {
  if ((a > (b - error)) && (a < (b + error))) {
    return true;
  }
  return false;
}

inline bool isUnblocked(const Map &map, Vec2<int> dest, Vec2<int> curr) {
  Vec2<int> direction = dest - curr;

  const int rowBit[] = {8, 0, 2};
  const int colBit[] = {1, 0, 4};

  if (std::abs(direction.x) > 1 || std::abs(direction.y) > 1)
    return false;

  int checkBit = rowBit[direction.x + 1] + colBit[direction.y + 1];
  int mapBit = map.getValue(curr.x, curr.y);

  if (checkBit & mapBit) {
    return false;
  }
  return true;
}

} // namespace Navigation

#endif // TESTING_FUNCTIONS_H