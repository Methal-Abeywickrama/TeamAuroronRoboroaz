#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "sensing.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>

namespace Navigation {

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================
namespace Config {
constexpr double CELL_SIZE = 0.25; // Cell size in meters
constexpr int GRID_ROWS = 12;      // Grid rows
constexpr int GRID_COLS = 12;      // Grid columns
constexpr double L_ERROR = 0.01;   // Cell center tolerance (10mm)

// Accuracy: 4 decimal points for angles (per user rule)
#ifndef M_PI
constexpr double PI = 3.1416;
#else
constexpr double PI = M_PI;
#endif
} // namespace Config

// ============================================================================
// 1. Vec2 TEMPLATE STRUCT
// ============================================================================
template <typename T> struct Vec2 {
  T x, y;
  Vec2() : x(0), y(0) {}
  Vec2(T x_val, T y_val) : x(x_val), y(y_val) {}

  Vec2 operator-(const Vec2 &other) const { return {x - other.x, y - other.y}; }
  Vec2 operator+(const Vec2 &other) const { return {x + other.x, y + other.y}; }
  bool operator==(const Vec2 &other) const {
    return x == other.x && y == other.y;
  }
  bool operator!=(const Vec2 &other) const { return !(*this == other); }
  bool operator<(const Vec2 &other) const {
    if (x != other.x)
      return x < other.x;
    return y < other.y;
  }
};

// ============================================================================
// 2. ODOMETRY CLASS
// ============================================================================
class Odometry {
private:
  double radius_;
  double axleLength_;
  double x_;     // Absolute X in meters (5 decimal accuracy)
  double y_;     // Absolute Y in meters (5 decimal accuracy)
  double theta_; // Heading in radians (4 decimal accuracy)
  double cellSize_;

public:
  // Default constructor
  Odometry()
      : radius_(0.0205), axleLength_(0.052), x_(0.0), y_(0.0), theta_(0.0),
        cellSize_(Config::CELL_SIZE) {}

  // Parameterized constructor
  Odometry(double r, double l, double cellSize = Config::CELL_SIZE)
      : radius_(r), axleLength_(l), x_(0.0), y_(0.0), theta_(0.0),
        cellSize_(cellSize) {}

  // Update position from encoder deltas
  void update(double leftRadDelta, double rightRadDelta, double dt) {
    // Convert encoder radians to linear distance
    double distLeft = radius_ * leftRadDelta;
    double distRight = radius_ * rightRadDelta;

    // Differential drive kinematics
    double d = (distLeft + distRight) / 2.0;
    double dTheta = (distRight - distLeft) / axleLength_;

    // Update position (5 decimal accuracy maintained)
    x_ += d * std::cos(theta_);
    y_ += d * std::sin(theta_);
    theta_ += dTheta;

    // Normalize angle to [-PI, PI]
    while (theta_ > Config::PI)
      theta_ -= 2.0 * Config::PI;
    while (theta_ <= -Config::PI)
      theta_ += 2.0 * Config::PI;
  }

  // Getters
  double getX() const { return x_; }
  double getY() const { return y_; }
  double getAngle() const { return theta_; }
  Vec2<double> getPos() const { return {x_, y_}; }

  // Convert meter coordinate to grid cell index
  int getGrid(double z) const {
    return static_cast<int>(std::round(z / cellSize_));
  }

  int getCellX() const { return getGrid(x_); }
  int getCellY() const { return getGrid(y_); }

  // Setters
  void setAngle(double ang) {
    theta_ = ang;
    while (theta_ > Config::PI)
      theta_ -= 2.0 * Config::PI;
    while (theta_ <= -Config::PI)
      theta_ += 2.0 * Config::PI;
  }

  void setPosition(double x, double y) {
    x_ = x;
    y_ = y;
  }

  void reset() {
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
  }
};

// ============================================================================
// 3. LOGGER CLASS
// ============================================================================
class Logger {
private:
  std::ofstream logFile_;
  bool isOpen_;

public:
  Logger() : isOpen_(false) {}

  Logger(const std::string &filename) : isOpen_(false) { open(filename); }

  ~Logger() {
    if (isOpen_) {
      logFile_.flush();
      logFile_.close();
    }
  }

  void open(const std::string &filename) {
    logFile_.open(filename, std::ios::out | std::ios::trunc);
    isOpen_ = logFile_.is_open();
    if (isOpen_) {
      logFile_ << "=== NAVIGATION LOG ===" << std::endl;
      logFile_ << std::setw(10) << "Time" << std::setw(12) << "AbsX"
               << std::setw(12) << "AbsY" << std::setw(8) << "CellX"
               << std::setw(8) << "CellY" << std::setw(10) << "Heading"
               << std::setw(8) << "Walls" << std::endl;
      logFile_ << std::string(68, '-') << std::endl;
    }
  }

  // Log position data with proper accuracy
  // Angles: 4 decimal, Distances: 5 decimal (per user rules)
  void logPosition(double time, double absX, double absY, int cellX, int cellY,
                   double heading, int walls) {
    if (!isOpen_)
      return;

    logFile_ << std::fixed << std::setw(10) << std::setprecision(3) << time
             << std::setw(12) << std::setprecision(5) << absX << std::setw(12)
             << std::setprecision(5) << absY << std::setw(8) << cellX
             << std::setw(8) << cellY << std::setw(10) << std::setprecision(4)
             << heading << "    ";

    // Wall bitmask as NESW string
    logFile_ << ((walls & 2) ? "N" : "-") << ((walls & 4) ? "E" : "-")
             << ((walls & 8) ? "S" : "-") << ((walls & 1) ? "W" : "-")
             << std::endl;
  }

  void logCheckpoint(int cellX, int cellY, double time) {
    if (!isOpen_)
      return;
    logFile_ << ">>> CHECKPOINT: Cell(" << cellX << "," << cellY
             << ") at Time=" << std::fixed << std::setprecision(3) << time
             << "s <<<" << std::endl;
  }

  void logPath(const std::vector<Vec2<int>> &path) {
    if (!isOpen_)
      return;
    logFile_ << ">>> A* PATH: ";
    for (const auto &p : path) {
      logFile_ << "(" << p.x << "," << p.y << ") ";
    }
    logFile_ << "<<<" << std::endl;
  }

  void logMessage(const std::string &msg) {
    if (!isOpen_)
      return;
    logFile_ << ">>> " << msg << " <<<" << std::endl;
  }

  // A* Pathfinding Logging Functions
  void logAStarStart(int startX, int startY, int destX, int destY,
                     double time) {
    if (!isOpen_)
      return;
    logFile_ << "\n" << std::string(50, '=') << "\n";
    logFile_ << ">>> A* PATHFINDING START <<<\n";
    logFile_ << "Time: " << std::fixed << std::setprecision(3) << time << "s\n";
    logFile_ << "Start Cell: (" << startX << ", " << startY << ")\n";
    logFile_ << "Destination: (" << destX << ", " << destY << ")\n";
    logFile_ << std::string(50, '-') << "\n";
  }

  void logCheckpointSummary(const std::vector<Vec2<int>> &checkpoints, int sumX,
                            int sumY, int destX, int destY) {
    if (!isOpen_)
      return;
    logFile_ << "Checkpoints found: " << checkpoints.size() << "\n";
    for (const auto &cp : checkpoints) {
      logFile_ << "  - (" << cp.x << ", " << cp.y << ")\n";
    }
    logFile_ << "Sum X: " << sumX << ", Sum Y: " << sumY << "\n";
    logFile_ << "Destination = (" << sumX << " mod 12, " << sumY
             << " mod 12) = (" << destX << ", " << destY << ")\n";
  }

  void logDirections(const std::vector<char> &directions) {
    if (!isOpen_)
      return;
    logFile_ << "Path directions: ";
    for (char d : directions)
      logFile_ << d << " ";
    logFile_ << "\nTotal moves: " << directions.size() << "\n";
  }

  void logPathStep(int stepNum, int totalSteps, char direction, int fromX,
                   int fromY, int toX, int toY) {
    if (!isOpen_)
      return;
    logFile_ << "Step " << stepNum << "/" << totalSteps << ": " << direction
             << " (" << fromX << "," << fromY << ") -> (" << toX << "," << toY
             << ")\n";
  }

  void logDestinationReached(int cellX, int cellY, double time) {
    if (!isOpen_)
      return;
    logFile_ << std::string(50, '=') << "\n";
    logFile_ << ">>> DESTINATION REACHED <<<\n";
    logFile_ << "Time: " << std::fixed << std::setprecision(3) << time << "s\n";
    logFile_ << "Final Cell: (" << cellX << ", " << cellY << ")\n";
    logFile_ << std::string(50, '=') << "\n\n";
  }

  void logAlreadyAtDestination(int cellX, int cellY, double time) {
    if (!isOpen_)
      return;
    logFile_ << "\n" << std::string(50, '=') << "\n";
    logFile_ << ">>> ALREADY AT DESTINATION <<<\n";
    logFile_ << "Time: " << std::fixed << std::setprecision(3) << time << "s\n";
    logFile_ << "Current Cell = Destination: (" << cellX << ", " << cellY
             << ")\n";
    logFile_ << std::string(50, '=') << "\n\n";
  }

  void flush() {
    if (isOpen_)
      logFile_.flush();
  }

  bool isOpen() const { return isOpen_; }
};

// ============================================================================
// 4. MAP CLASS
// ============================================================================
class Map {
private:
  int rows_;
  int cols_;
  Vec2<int> origin_;
  double cellSize_;
  std::vector<int> map_;      // Wall bitmask storage
  std::vector<bool> visited_; // Visited tracking for DFS exploration
  std::vector<Vec2<int>> checkpoints_;
  int checkpointSumX_;
  int checkpointSumY_;
  Odometry odom_;

public:
  // Default constructor
  Map()
      : rows_(Config::GRID_ROWS), cols_(Config::GRID_COLS), origin_({0, 0}),
        cellSize_(Config::CELL_SIZE), checkpointSumX_(0), checkpointSumY_(0) {
    map_.resize(rows_ * cols_, 0);
    visited_.resize(rows_ * cols_, false);
  }

  // Parameterized constructor
  Map(int rows, int cols, Vec2<int> origin, double cellSize)
      : rows_(rows), cols_(cols), origin_(origin), cellSize_(cellSize),
        checkpointSumX_(0), checkpointSumY_(0) {
    map_.resize(rows_ * cols_, 0);
    visited_.resize(rows_ * cols_, false);
  }

  // Constructor with Odometry
  Map(int rows, int cols, Vec2<int> origin, double cellSize,
      const Odometry &odom)
      : rows_(rows), cols_(cols), origin_(origin), cellSize_(cellSize),
        checkpointSumX_(0), checkpointSumY_(0), odom_(odom) {
    map_.resize(rows_ * cols_, 0);
    visited_.resize(rows_ * cols_, false);
  }

  // Index calculation
  int index(int x, int y) const { return y * cols_ + x; }

  // Bounds check
  bool isValid(int x, int y) const {
    return x >= 0 && x < cols_ && y >= 0 && y < rows_;
  }

  // -------------------------------------------------------------------------
  // VISITED TRACKING (for DFS exploration)
  // -------------------------------------------------------------------------
  bool isVisited(int x, int y) const {
    if (!isValid(x, y))
      return true; // Out of bounds = "visited"
    return visited_[index(x, y)];
  }

  void setVisited(int x, int y, bool val = true) {
    if (!isValid(x, y))
      return;
    visited_[index(x, y)] = val;
  }

  int countVisited() const {
    int count = 0;
    for (bool v : visited_) {
      if (v)
        count++;
    }
    return count;
  }

  // Get wall bitmask for cell
  int getValue(int x, int y) const {
    if (!isValid(x, y))
      return 15; // All walls if out of bounds
    return map_[index(x, y)];
  }

  // Set wall bitmask directly
  void setValue(int x, int y, int walls) {
    if (!isValid(x, y))
      return;
    map_[index(x, y)] = walls;
  }

  // Update cell from boolean wall array [W, N, E, S]
  // Wall encoding: W=1, N=2, E=4, S=8
  void updateCell(bool walls[4], int x, int y) {
    if (!isValid(x, y))
      return;
    int i = index(x, y);
    int wallMask = (walls[0] ? 1 : 0) + // West
                   (walls[1] ? 2 : 0) + // North
                   (walls[2] ? 4 : 0) + // East
                   (walls[3] ? 8 : 0);  // South

    if (map_[i] == 0) {
      map_[i] = wallMask;
    }
  }

  // Transform sensor-relative walls to absolute based on heading
  // Sensor input: Front, Right, Back, Left (robot-relative)
  // Output: N, E, S, W (absolute based on heading)
  void updateFromSensors(bool front, bool right, bool back, bool left,
                         double heading, int cellX, int cellY) {
    if (!isValid(cellX, cellY))
      return;
    if (map_[index(cellX, cellY)] != 0)
      return; // Already mapped

    // Determine robot heading direction (0=N, 1=E, 2=S, 3=W)
    // Use 4 decimal precision for angle calculation (per user rule)
    int headDir = static_cast<int>(std::round(heading / (Config::PI / 2.0)));
    headDir = ((headDir % 4) + 4) % 4; // Normalize to 0-3

    // Sensor array: [Front, Right, Back, Left]
    bool sensors[4] = {front, right, back, left};

    // Map to absolute directions based on heading
    // headDir=0 (North): Front=N, Right=E, Back=S, Left=W
    // headDir=1 (East):  Front=E, Right=S, Back=W, Left=N
    // etc.
    bool absWalls[4]; // [W, N, E, S]
    for (int i = 0; i < 4; i++) {

      // Convert from sensor index to absolute wall index
      // Sensor: 0=Front, 1=Right, 2=Back, 3=Left -> N,E,S,W based on
      // headDir
      switch ((i + headDir) % 4) {
      case 0:
        absWalls[1] = sensors[i];
        break; // North
      case 1:
        absWalls[2] = sensors[i];
        break; // East
      case 2:
        absWalls[3] = sensors[i];
        break; // South
      case 3:
        absWalls[0] = sensors[i];
        break; // West
      }
    }

    updateCell(absWalls, cellX, cellY);
  }

  // Check if at cell center and update map
  void updateMapIfAtCenter(bool front, bool right, bool back, bool left,
                           double posX, double posY, double heading) {
    // Check if near cell center (within L_ERROR tolerance)
    double xMod = std::fmod(std::abs(posX), cellSize_);
    double yMod = std::fmod(std::abs(posY), cellSize_);

    // Account for values near cellSize_ (wrapping)
    if (xMod > cellSize_ - Config::L_ERROR)
      xMod = cellSize_ - xMod;
    if (yMod > cellSize_ - Config::L_ERROR)
      yMod = cellSize_ - yMod;

    if (xMod < Config::L_ERROR && yMod < Config::L_ERROR) {
      int cellX = static_cast<int>(std::round(posX / cellSize_));
      int cellY = static_cast<int>(std::round(posY / cellSize_));
      updateFromSensors(front, right, back, left, heading, cellX, cellY);
    }
  }

  // Checkpoint management
  void addCheckpoint(int x, int y) {
    Vec2<int> point = {x, y};
    for (const auto &cp : checkpoints_) {
      if (cp == point)
        return; // Already recorded
    }
    checkpoints_.push_back(point);
    checkpointSumX_ += x;
    checkpointSumY_ += y;
  }

  Vec2<int> getDestination() const {
    if (checkpoints_.empty())
      return {0, 0};
    int destX = checkpointSumX_ % cols_;
    int destY = checkpointSumY_ % rows_;
    return {destX, destY};
  }

  const std::vector<Vec2<int>> &getCheckpoints() const { return checkpoints_; }
  int getCheckpointSumX() const { return checkpointSumX_; }
  int getCheckpointSumY() const { return checkpointSumY_; }

  // Getters
  int getRows() const { return rows_; }
  int getCols() const { return cols_; }
  Vec2<int> getDim() const { return {rows_, cols_}; }
  Odometry &getOdometry() { return odom_; }
  const Odometry &getOdometry() const { return odom_; }

  // Debug print
  void printMap() const {
    std::cout << "Map (" << rows_ << "x" << cols_ << "):" << std::endl;
    for (int y = rows_ - 1; y >= 0; y--) {
      for (int x = 0; x < cols_; x++) {
        std::cout << std::setw(3) << map_[index(x, y)];
      }
      std::cout << std::endl;
    }
  }
};

// ============================================================================
// 5. A* PATHFINDER CLASS
// ============================================================================
class PathFinder {
private:
  const Map *map_;

  // A* node for priority queue
  struct Node {
    Vec2<int> pos;
    double f; // f = g + h

    bool operator>(const Node &other) const { return f > other.f; }
  };

  // Manhattan distance heuristic (no diagonals)
  double heuristic(Vec2<int> a, Vec2<int> b) const {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
  }

  // Check if movement from curr to next is blocked by wall
  // Coordinate system: X increases LEFT (West), Y increases UP (North)
  bool isBlocked(Vec2<int> curr, Vec2<int> next) const {
    Vec2<int> dir = next - curr;

    // Must be adjacent (no diagonals)
    if (std::abs(dir.x) + std::abs(dir.y) != 1)
      return true;

    int walls = map_->getValue(curr.x, curr.y);

    // Check wall in direction of movement
    // Wall encoding: W=1, N=2, E=4, S=8
    if (dir.y == 1 && (walls & 2))
      return true; // North wall (dy = +1)
    if (dir.x == -1 && (walls & 4))
      return true; // East wall (dx = -1, X decreases = East)
    if (dir.y == -1 && (walls & 8))
      return true; // South wall (dy = -1)
    if (dir.x == 1 && (walls & 1))
      return true; // West wall (dx = +1, X increases = West)

    return false;
  }

public:
  PathFinder() : map_(nullptr) {}
  PathFinder(const Map *map) : map_(map) {}

  void setMap(const Map *map) { map_ = map; }

  // Find path from start to goal using A*
  // Returns vector of cell coordinates, empty if no path found
  std::vector<Vec2<int>> findPath(Vec2<int> start, Vec2<int> goal) {
    std::vector<Vec2<int>> path;
    if (!map_)
      return path;
    if (!map_->isValid(start.x, start.y) || !map_->isValid(goal.x, goal.y)) {
      return path;
    }

    // Priority queue (min-heap by f value)
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;

    // Closed set
    std::set<Vec2<int>> closedSet;

    // Parent tracking for path reconstruction
    std::map<Vec2<int>, Vec2<int>> cameFrom;

    // g-score: cost from start
    std::map<Vec2<int>, double> gScore;

    // Initialize
    gScore[start] = 0.0;
    openSet.push({start, heuristic(start, goal)});

    // 4-directional neighbors (N, E, S, W)
    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {1, 0, -1, 0};

    while (!openSet.empty()) {
      Node current = openSet.top();
      openSet.pop();

      if (current.pos == goal) {
        // Reconstruct path
        Vec2<int> node = goal;
        while (node != start) {
          path.push_back(node);
          node = cameFrom[node];
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
      }

      if (closedSet.count(current.pos))
        continue;
      closedSet.insert(current.pos);

      // Explore neighbors
      for (int i = 0; i < 4; i++) {
        Vec2<int> neighbor = {current.pos.x + dx[i], current.pos.y + dy[i]};

        if (!map_->isValid(neighbor.x, neighbor.y))
          continue;
        if (closedSet.count(neighbor))
          continue;
        if (isBlocked(current.pos, neighbor))
          continue;

        double tentativeG = gScore[current.pos] + 1.0;

        if (gScore.find(neighbor) == gScore.end() ||
            tentativeG < gScore[neighbor]) {
          cameFrom[neighbor] = current.pos;
          gScore[neighbor] = tentativeG;
          double f = tentativeG + heuristic(neighbor, goal);
          openSet.push({neighbor, f});
        }
      }
    }

    return path; // Empty = no path found
  }

  // Convert path to direction sequence
  // Returns: 'N'=North, 'E'=East, 'S'=South, 'W'=West
  // Coordinate system: X increases LEFT (West), Y increases UP (North)
  std::vector<char> pathToDirections(const std::vector<Vec2<int>> &path) {
    std::vector<char> directions;
    for (size_t i = 1; i < path.size(); i++) {
      Vec2<int> dir = path[i] - path[i - 1];
      if (dir.y == 1)
        directions.push_back('N'); // North (dy = +1)
      else if (dir.x == -1)
        directions.push_back('E'); // East (dx = -1, X decreases)
      else if (dir.y == -1)
        directions.push_back('S'); // South (dy = -1)
      else if (dir.x == 1)
        directions.push_back('W'); // West (dx = +1, X increases)
    }
    return directions;
  }
};

// ============================================================================
// 6. NAVIGATOR CLASS (UNIFIED WRAPPER)
// ============================================================================
class Navigator {
private:
  Map map_;
  Logger logger_;
  PathFinder pathfinder_;
  double lastLogTime_;
  double logInterval_; // Minimum time between logs

public:
  Navigator() : lastLogTime_(0.0), logInterval_(0.1) {
    pathfinder_.setMap(&map_);
  }

  Navigator(const Map &m) : map_(m), lastLogTime_(0.0), logInterval_(0.1) {
    pathfinder_.setMap(&map_);
  }

  ~Navigator() {}

  // Initialize logger
  void initLogger(const std::string &filename = "navigation_log.txt") {
    logger_.open(filename);
  }

  // Access map
  Map &getMap() { return map_; }
  const Map &getMap() const { return map_; }
  void setMap(const Map &m) {
    map_ = m;
    pathfinder_.setMap(&map_);
  }

  // Access odometry through map
  Odometry &getOdometry() { return map_.getOdometry(); }

  // Check for green wall checkpoint and record
  void checkAndRecordCheckpoint(Sensor::Sensing *sensors, double time) {
    if (!sensors)
      return;

    if (sensors->isFrontWallGreen()) {
      Odometry &odom = map_.getOdometry();
      int cellX = odom.getCellX();
      int cellY = odom.getCellY();

      map_.addCheckpoint(cellX, cellY);
      logger_.logCheckpoint(cellX, cellY, time);
      logger_.flush();
    }
  }

  // Log current state
  void logCurrentState(double time, bool front, bool right, bool back,
                       bool left) {
    if (time - lastLogTime_ < logInterval_)
      return;

    Odometry &odom = map_.getOdometry();
    int cellX = odom.getCellX();
    int cellY = odom.getCellY();
    int walls = map_.getValue(cellX, cellY);

    logger_.logPosition(time, odom.getX(), odom.getY(), cellX, cellY,
                        odom.getAngle(), walls);
    lastLogTime_ = time;
  }

  // Update map from sensor readings
  // Uses distance thresholds from methal_sensing.h
  void updateMap(Sensor::Sensing *sensors, double time) {
    if (!sensors)
      return;

    // Wall detection using distance thresholds (0.09m = wall present)
    // Front: average of PS0 and PS7
    // Right: PS2, Left: PS5
    constexpr double WALL_THRESHOLD = 0.09; // 90mm wall detection

    double frontDist = sensors->getDistanceToFrontWall();
    double rightDist = sensors->getDistanceToRightWall();
    double leftDist = sensors->getDistanceToLeftWall();

    bool front = (frontDist < WALL_THRESHOLD);
    bool right = (rightDist < WALL_THRESHOLD);
    bool left = (leftDist < WALL_THRESHOLD);
    bool back = false; // E-puck has no reliable back sensor

    Odometry &odom = map_.getOdometry();
    map_.updateMapIfAtCenter(front, right, back, left, odom.getX(), odom.getY(),
                             odom.getAngle());

    logCurrentState(time, front, right, back, left);
  }

  // Get destination from checkpoints
  Vec2<int> getDestination() const { return map_.getDestination(); }

  // Compute path to destination
  std::vector<Vec2<int>> getPathToDestination() {
    Odometry &odom = map_.getOdometry();
    Vec2<int> start = {odom.getCellX(), odom.getCellY()};
    Vec2<int> goal = map_.getDestination();

    auto path = pathfinder_.findPath(start, goal);
    if (!path.empty()) {
      logger_.logPath(path);
    }
    return path;
  }

  // Get path as direction sequence
  std::vector<char> getDirectionsToDestination() {
    auto path = getPathToDestination();
    return pathfinder_.pathToDirections(path);
  }

  // Find path between arbitrary points
  std::vector<Vec2<int>> findPath(Vec2<int> start, Vec2<int> goal) {
    return pathfinder_.findPath(start, goal);
  }

  // Utility: Check if cell is valid
  bool isValid(int x, int y) const { return map_.isValid(x, y); }

  // Check if movement is unblocked
  // Coordinate system: X increases LEFT (West), Y increases UP (North)
  bool isUnblocked(Vec2<int> curr, Vec2<int> dest) const {
    Vec2<int> dir = dest - curr;

    if (std::abs(dir.x) > 1 || std::abs(dir.y) > 1)
      return false;
    if (std::abs(dir.x) + std::abs(dir.y) != 1)
      return false;

    int walls = map_.getValue(curr.x, curr.y);

    // Wall encoding: W=1, N=2, E=4, S=8
    if (dir.y == 1 && (walls & 2))
      return false; // North blocked (dy = +1)
    if (dir.x == -1 && (walls & 4))
      return false; // East blocked (dx = -1, X decreases = East)
    if (dir.y == -1 && (walls & 8))
      return false; // South blocked (dy = -1)
    if (dir.x == 1 && (walls & 1))
      return false; // West blocked (dx = +1, X increases = West)

    return true;
  }

  // Debug
  void debugPrintMap() { map_.printMap(); }

  void logMessage(const std::string &msg) { logger_.logMessage(msg); }

  void flushLog() { logger_.flush(); }

  // A* Navigation Logging Wrappers
  void logAStarStart(int startX, int startY, int destX, int destY,
                     double time) {
    logger_.logAStarStart(startX, startY, destX, destY, time);
  }

  void logCheckpointSummary() {
    auto checkpoints = map_.getCheckpoints();
    int sumX = map_.getCheckpointSumX();
    int sumY = map_.getCheckpointSumY();
    auto dest = map_.getDestination();
    logger_.logCheckpointSummary(checkpoints, sumX, sumY, dest.x, dest.y);
  }

  void logPath(const std::vector<Vec2<int>> &path) { logger_.logPath(path); }

  void logDirections(const std::vector<char> &directions) {
    logger_.logDirections(directions);
  }

  void logPathStep(int stepNum, int totalSteps, char direction, int fromX,
                   int fromY, int toX, int toY) {
    logger_.logPathStep(stepNum, totalSteps, direction, fromX, fromY, toX, toY);
  }

  void logDestinationReached(int cellX, int cellY, double time) {
    logger_.logDestinationReached(cellX, cellY, time);
  }

  void logAlreadyAtDestination(int cellX, int cellY, double time) {
    logger_.logAlreadyAtDestination(cellX, cellY, time);
  }

  // Expose pathToDirections
  std::vector<char> pathToDirections(const std::vector<Vec2<int>> &path) {
    return pathfinder_.pathToDirections(path);
  }
};

} // namespace Navigation

#endif // NAVIGATOR_H