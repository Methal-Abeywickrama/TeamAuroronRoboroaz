#ifndef EXPLORER_H
#define EXPLORER_H

// =============================================================================
// DFS MAZE EXPLORER
// Implements DFS exploration with green wall checkpoint detection
// Coordinate: X increases LEFT, Y increases UP, start at (0,0) bottom-right
// =============================================================================

#include "motion_control.h"
#include "navigator.h"
#include "sensing.h"
#include <iostream>
#include <sstream>
#include <stack>

namespace Exploration {

// Direction enum (absolute directions in the maze)
enum class Heading { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

// Direction deltas: X increases LEFT, Y increases UP
constexpr int DX[4] = {0, -1, 0, 1}; // N, E, S, W
constexpr int DY[4] = {1, 0, -1, 0}; // N, E, S, W

// Explorer state machine
enum class ExplorerState {
  Idle,
  MovingForward,        // startFastForward active
  LoggingCell,          // Record wall data
  CheckingFrontGreen,   // Check front wall green
  TurningToLeft,        // Turning left
  CheckingLeftGreen,    // Check left wall green
  TurningBackFromLeft,  // Turn right to face forward
  TurningToRight1,      // First right turn
  TurningToRight2,      // Second right turn (now facing right)
  CheckingRightGreen,   // Check right wall green
  TurningBackFromRight, // Turn left to face forward
  FindingNextCell,      // DFS: select next direction
  TurningToNextCell,    // Turn to face next cell (single turn)
  TurningToNextCell2,   // Second turn for 180° (two turnLeft)
  Backtracking,         // Return to previous cell
  Complete,             // DFS exploration done

  // Destination Navigation States
  CalculatingPath,   // Compute A* path to destination
  TurningToPath,     // Turn to face next direction in path
  TurningToPath2,    // Second turn for 180° rotations
  MovingAlongPath,   // Moving forward to next cell on path
  DestinationReached // Final state - robot at destination
};

// =============================================================================
// EXPLORER CLASS
// =============================================================================
class Explorer {
private:
  webots::Robot *robot_;
  Sensor::Sensing *sensing_;
  Motion::MovingController *motion_;
  Navigation::Navigator *navigator_;

  ExplorerState state_;
  Heading currentHeading_;
  int currentX_, currentY_;
  std::stack<Navigation::Vec2<int>> explorationStack_;
  Heading nextMoveDirection_;     // Direction for next move
  bool pendingFrontWall_ = false; // Flag: add front wall after next scan
  bool isBacktracking_ =
      false; // Flag: skip checkpoint detection for visited cells

  // Path execution variables (for destination navigation)
  std::vector<char> pathDirections_;
  size_t pathIndex_ = 0;
  Navigation::Vec2<int> destination_;

  static constexpr int GRID_SIZE = 12;

  // -------------------------------------------------------------------------
  // HELPER: Get wall bit for direction
  // Wall encoding: W=1, N=2, E=4, S=8
  // -------------------------------------------------------------------------
  int headingToWallBit(Heading h) const {
    switch (h) {
    case Heading::NORTH:
      return 2;
    case Heading::EAST:
      return 4;
    case Heading::SOUTH:
      return 8;
    case Heading::WEST:
      return 1;
    }
    return 0;
  }

  // -------------------------------------------------------------------------
  // HELPER: Check if direction blocked by wall
  // -------------------------------------------------------------------------
  bool isBlocked(Heading dir) const {
    int walls = navigator_->getMap().getValue(currentX_, currentY_);
    return (walls & headingToWallBit(dir)) != 0;
  }

  // -------------------------------------------------------------------------
  // HELPER: Check if can move to direction (valid, unblocked, unvisited)
  // -------------------------------------------------------------------------
  bool canMoveTo(Heading dir) const {
    int nx = currentX_ + DX[static_cast<int>(dir)];
    int ny = currentY_ + DY[static_cast<int>(dir)];

    if (nx < 0 || nx >= GRID_SIZE || ny < 0 || ny >= GRID_SIZE)
      return false;
    if (isBlocked(dir))
      return false;
    if (navigator_->getMap().isVisited(nx, ny))
      return false;

    return true;
  }

  // -------------------------------------------------------------------------
  // SCAN AND RECORD WALLS at current position
  // -------------------------------------------------------------------------
  void scanAndRecordWalls() {
    if (!sensing_ || !navigator_)
      return;

    sensing_->update();

    bool front = sensing_->isWallAtFront();
    bool left = sensing_->isWallAtLeft();
    bool right = sensing_->isWallAtRight();
    bool back = false;

    // Map sensor directions to absolute directions based on heading
    bool absWalls[4] = {false, false, false, false}; // W, N, E, S

    int h = static_cast<int>(currentHeading_);

    // Convert heading to wall array index
    auto headingToWallIdx = [](int heading) -> int {
      switch (heading) {
      case 0:
        return 1; // N
      case 1:
        return 2; // E
      case 2:
        return 3; // S
      case 3:
        return 0; // W
      }
      return 0;
    };

    absWalls[headingToWallIdx(h)] = front;
    absWalls[headingToWallIdx((h + 1) % 4)] = right;
    absWalls[headingToWallIdx((h + 2) % 4)] = back;
    absWalls[headingToWallIdx((h + 3) % 4)] = left;

    navigator_->getMap().updateCell(absWalls, currentX_, currentY_);
    navigator_->getMap().setVisited(currentX_, currentY_);

    std::cout << "Cell(" << currentX_ << "," << currentY_ << ") walls=";
    std::cout << (absWalls[0] ? "W" : "-") << (absWalls[1] ? "N" : "-")
              << (absWalls[2] ? "E" : "-") << (absWalls[3] ? "S" : "-")
              << std::endl;
  }

  // -------------------------------------------------------------------------
  // CHECK if in destination navigation mode (CalculatingPath or after)
  // -------------------------------------------------------------------------
  bool isInDestinationNavigation() const {
    return state_ == ExplorerState::CalculatingPath ||
           state_ == ExplorerState::TurningToPath ||
           state_ == ExplorerState::TurningToPath2 ||
           state_ == ExplorerState::MovingAlongPath ||
           state_ == ExplorerState::DestinationReached;
  }

  // -------------------------------------------------------------------------
  // LOG CHECKPOINT at current cell
  // -------------------------------------------------------------------------
  void logCheckpoint() {
    // Skip checkpoint logging during destination navigation
    if (isInDestinationNavigation()) {
      return;
    }
    navigator_->getMap().addCheckpoint(currentX_, currentY_);
    std::cout << ">>> CHECKPOINT at (" << currentX_ << "," << currentY_
              << ") <<<" << std::endl;
  }

  // -------------------------------------------------------------------------
  // GET NUMBER OF TURNS needed to face target direction
  // Returns: 0=none, 1=right, 2=back, 3=left (or -1=left)
  // -------------------------------------------------------------------------
  int turnsNeeded(Heading target) const {
    return (static_cast<int>(target) - static_cast<int>(currentHeading_) + 4) %
           4;
  }

  // -------------------------------------------------------------------------
  // PATH EXECUTION HELPERS
  // -------------------------------------------------------------------------
  // Convert direction char to Heading enum
  Heading charToHeading(char dir) const {
    switch (dir) {
    case 'N':
      return Heading::NORTH;
    case 'E':
      return Heading::EAST;
    case 'S':
      return Heading::SOUTH;
    case 'W':
      return Heading::WEST;
    default:
      return Heading::NORTH;
    }
  }

  // Get delta X for direction (X increases LEFT = West)
  int getDX(char dir) const {
    switch (dir) {
    case 'N':
      return 0;
    case 'E':
      return -1; // East = decreasing X
    case 'S':
      return 0;
    case 'W':
      return 1; // West = increasing X
    default:
      return 0;
    }
  }

  // Get delta Y for direction (Y increases UP = North)
  int getDY(char dir) const {
    switch (dir) {
    case 'N':
      return 1; // North = increasing Y
    case 'E':
      return 0;
    case 'S':
      return -1; // South = decreasing Y
    case 'W':
      return 0;
    default:
      return 0;
    }
  }

public:
  // -------------------------------------------------------------------------
  // CONSTRUCTOR
  // -------------------------------------------------------------------------
  Explorer(webots::Robot *robot, Sensor::Sensing *sensing,
           Motion::MovingController *motion, Navigation::Navigator *navigator)
      : robot_(robot), sensing_(sensing), motion_(motion),
        navigator_(navigator), state_(ExplorerState::Idle),
        currentHeading_(Heading::NORTH), currentX_(0), currentY_(0),
        nextMoveDirection_(Heading::NORTH) {}

  // -------------------------------------------------------------------------
  // START EXPLORATION
  // -------------------------------------------------------------------------
  void start() {
    state_ = ExplorerState::LoggingCell; // Start by logging first cell
    currentX_ = 0;
    currentY_ = 0;
    currentHeading_ = Heading::NORTH;

    while (!explorationStack_.empty())
      explorationStack_.pop();
    explorationStack_.push({currentX_, currentY_});

    std::cout << "=== DFS EXPLORATION STARTED ===" << std::endl;
    std::cout << "Start: cell(0,0), facing NORTH" << std::endl;
  }

  // -------------------------------------------------------------------------
  // UPDATE - call every timestep
  // Returns true when exploration complete
  // -------------------------------------------------------------------------
  bool update(double dt) {
    if (state_ == ExplorerState::Complete || state_ == ExplorerState::Idle)
      return state_ == ExplorerState::Complete;

    // If motion is busy (turning), wait for it
    if (motion_->isBusy()) {
      motion_->update(dt);
      return false;
    }

    // If fast forward is active, wait for it
    if (motion_->isFastDriveActive()) {
      if (motion_->updateFastForward(dt)) {
        // Check if wall failsafe occurred
        if (motion_->wasWallFailsafe()) {
          double distTraveled = motion_->getFailsafeDistance();

          if (distTraveled < 0.20) {
            // EARLY failsafe (< 0.20m): Real wall in path
            // Stay at current cell, correct position, clear data, re-scan
            std::cout << "=== EARLY WALL FAILSAFE (dist=" << distTraveled
                      << "m) ===" << std::endl;
            std::cout << "Real wall detected - rescanning cell(" << currentX_
                      << "," << currentY_ << ")" << std::endl;

            // Correct robot position to 8.5cm from wall
            motion_->correctToWall(0.085);

            // For green walls: sensor unreliable, back up 3.5cm extra
            // and set flag to add front wall AFTER scan (sensor unreliable at
            // >9cm)
            sensing_->update();
            if (sensing_->isFrontWallGreen()) {
              std::cout << "Green wall - backing up 3.5cm" << std::endl;
              motion_->moveBackward(0.035);
              pendingFrontWall_ = true; // Add front wall after next scan
            }

            // Clear current cell data (previous scan missed the wall)
            navigator_->getMap().setValue(currentX_, currentY_, 0);
            navigator_->getMap().setVisited(currentX_, currentY_, false);

            // Re-scan this cell (now we can detect the wall we missed)
            state_ = ExplorerState::LoggingCell;
          } else {
            // LATE failsafe (>= 0.20m): Already near destination
            // Treat as normal arrival at next cell
            std::cout << "=== LATE WALL FAILSAFE (dist=" << distTraveled
                      << "m) ===" << std::endl;
            currentX_ += DX[static_cast<int>(currentHeading_)];
            currentY_ += DY[static_cast<int>(currentHeading_)];
            std::cout << "Treating as arrival at cell(" << currentX_ << ","
                      << currentY_ << ")" << std::endl;
            state_ = ExplorerState::LoggingCell;
          }
        } else {
          // Normal completion - update position
          currentX_ += DX[static_cast<int>(currentHeading_)];
          currentY_ += DY[static_cast<int>(currentHeading_)];
          std::cout << "Arrived at cell(" << currentX_ << "," << currentY_
                    << ")" << std::endl;

          // Check if we're in path execution mode
          if (state_ == ExplorerState::MovingAlongPath) {
            // Log the step
            char dir = pathDirections_[pathIndex_];
            navigator_->logPathStep(pathIndex_ + 1, pathDirections_.size(), dir,
                                    currentX_ - getDX(dir),
                                    currentY_ - getDY(dir), currentX_,
                                    currentY_);

            std::cout << "Step " << (pathIndex_ + 1) << "/"
                      << pathDirections_.size() << ": " << dir << " -> ("
                      << currentX_ << "," << currentY_ << ")" << std::endl;

            pathIndex_++;

            // Check if we've reached destination
            if (pathIndex_ >= pathDirections_.size()) {
              navigator_->logDestinationReached(currentX_, currentY_,
                                                robot_->getTime());
              navigator_->flushLog();
              std::cout << "\n=== DESTINATION REACHED! ===" << std::endl;
              std::cout << "Final position: (" << currentX_ << "," << currentY_
                        << ")" << std::endl;
              state_ = ExplorerState::DestinationReached;
              return true;
            } else {
              // Continue to next step
              state_ = ExplorerState::TurningToPath;
            }
          } else {
            // Normal exploration mode
            state_ = ExplorerState::LoggingCell;
          }
        }
      }
      return false;
    }

    switch (state_) {
    // ----- LOGGING CELL -----
    case ExplorerState::LoggingCell:
      scanAndRecordWalls();
      // Add pending front wall bit AFTER scan (for green walls)
      if (pendingFrontWall_) {
        int frontWallBit = headingToWallBit(currentHeading_);
        int currentWalls = navigator_->getMap().getValue(currentX_, currentY_);
        int newWalls = currentWalls | frontWallBit;
        navigator_->getMap().setValue(currentX_, currentY_, newWalls);

        // Log checkpoint - we already know it's green
        logCheckpoint();

        std::cout << "Cell(" << currentX_ << "," << currentY_
                  << ") walls=" << ((newWalls & 1) ? "W" : "-")
                  << ((newWalls & 2) ? "N" : "-")
                  << ((newWalls & 4) ? "E" : "-")
                  << ((newWalls & 8) ? "S" : "-") << " (green wall checkpoint)"
                  << std::endl;

        pendingFrontWall_ = false;
        state_ = ExplorerState::FindingNextCell; // Skip checkpoint detection
      } else if (isBacktracking_) {
        // Skip checkpoint detection for visited cells
        std::cout << "Backtracked - skipping checkpoint detection" << std::endl;
        isBacktracking_ = false;
        state_ = ExplorerState::FindingNextCell;
      } else {
        state_ = ExplorerState::CheckingFrontGreen; // Normal flow
      }
      break;

    // ----- CHECK FRONT GREEN -----
    case ExplorerState::CheckingFrontGreen: {
      sensing_->update();
      bool wallAtFront = sensing_->isWallAtFront();
      bool frontIsGreen = sensing_->isFrontWallGreen();

      // Debug log to exploration_log.txt
      std::ostringstream debugMsg;
      debugMsg << "GREEN_CHECK FRONT @ (" << currentX_ << "," << currentY_
               << ") "
               << "heading=" << getHeadingStr() << " "
               << "wall=" << (wallAtFront ? "YES" : "NO") << " "
               << "green=" << (frontIsGreen ? "YES" : "NO") << " "
               << "raw0=" << sensing_->getRawDistance(0) << " "
               << "raw7=" << sensing_->getRawDistance(7);
      navigator_->logMessage(debugMsg.str());
      navigator_->flushLog();

      // Trust camera alone: if camera sees green (>50% coverage), log
      // checkpoint Distance sensors have limited range and may miss walls
      if (frontIsGreen) {
        logCheckpoint();
        state_ = ExplorerState::FindingNextCell; // Skip left/right check
      } else {
        state_ = ExplorerState::TurningToLeft;
        motion_->turnLeft();
      }
      break;
    }

    // ----- TURN LEFT -----
    case ExplorerState::TurningToLeft:
      // Turn complete (isBusy was false)
      currentHeading_ =
          static_cast<Heading>((static_cast<int>(currentHeading_) + 3) % 4);
      state_ = ExplorerState::CheckingLeftGreen;
      break;

    // ----- CHECK LEFT GREEN -----
    case ExplorerState::CheckingLeftGreen: {
      sensing_->update();
      bool wallAtFront = sensing_->isWallAtFront();
      bool frontIsGreen = sensing_->isFrontWallGreen();

      // Debug log to exploration_log.txt
      std::ostringstream debugMsg;
      debugMsg << "GREEN_CHECK LEFT @ (" << currentX_ << "," << currentY_
               << ") "
               << "heading=" << getHeadingStr() << " "
               << "wall=" << (wallAtFront ? "YES" : "NO") << " "
               << "green=" << (frontIsGreen ? "YES" : "NO") << " "
               << "raw0=" << sensing_->getRawDistance(0) << " "
               << "raw7=" << sensing_->getRawDistance(7);
      navigator_->logMessage(debugMsg.str());
      navigator_->flushLog();

      // Trust camera alone: if camera sees green (>50% coverage), log
      // checkpoint Distance sensors have limited range and may miss walls at
      // turning distance
      if (frontIsGreen) {
        logCheckpoint();
      }
      state_ = ExplorerState::TurningBackFromLeft;
      motion_->turnRight(); // Face forward again
      break;
    }

    // ----- TURN BACK FROM LEFT -----
    case ExplorerState::TurningBackFromLeft:
      currentHeading_ =
          static_cast<Heading>((static_cast<int>(currentHeading_) + 1) % 4);
      state_ = ExplorerState::TurningToRight1;
      motion_->turnRight(); // Turn right to face right side
      break;

    // ----- TURN TO RIGHT (single turn from front to right) -----
    case ExplorerState::TurningToRight1:
      currentHeading_ =
          static_cast<Heading>((static_cast<int>(currentHeading_) + 1) % 4);
      state_ = ExplorerState::CheckingRightGreen; // Go directly to check
      break;

    // ----- (TurningToRight2 removed - was causing robot to face back) -----

    // ----- CHECK RIGHT GREEN -----
    case ExplorerState::CheckingRightGreen: {
      sensing_->update();
      bool wallAtFront = sensing_->isWallAtFront();
      bool frontIsGreen = sensing_->isFrontWallGreen();

      // Debug log to exploration_log.txt
      std::ostringstream debugMsg;
      debugMsg << "GREEN_CHECK RIGHT @ (" << currentX_ << "," << currentY_
               << ") "
               << "heading=" << getHeadingStr() << " "
               << "wall=" << (wallAtFront ? "YES" : "NO") << " "
               << "green=" << (frontIsGreen ? "YES" : "NO") << " "
               << "raw0=" << sensing_->getRawDistance(0) << " "
               << "raw7=" << sensing_->getRawDistance(7);
      navigator_->logMessage(debugMsg.str());
      navigator_->flushLog();

      // Trust camera alone: if camera sees green (>50% coverage), log
      // checkpoint Distance sensors have limited range and may miss walls at
      // turning distance
      if (frontIsGreen) {
        logCheckpoint();
      }
      state_ = ExplorerState::TurningBackFromRight;
      motion_->turnLeft(); // Face forward again
      break;
    }

    // ----- TURN BACK FROM RIGHT -----
    case ExplorerState::TurningBackFromRight:
      currentHeading_ =
          static_cast<Heading>((static_cast<int>(currentHeading_) + 3) % 4);
      state_ = ExplorerState::FindingNextCell;
      break;

    // ----- FIND NEXT CELL (DFS) -----
    case ExplorerState::FindingNextCell: {
      // Priority: NORTH, WEST, EAST, SOUTH
      Heading priority[4] = {Heading::NORTH, Heading::WEST, Heading::EAST,
                             Heading::SOUTH};
      bool found = false;

      for (Heading dir : priority) {
        if (canMoveTo(dir)) {
          nextMoveDirection_ = dir;
          explorationStack_.push({currentX_, currentY_});

          // Turn to face direction
          int turns = turnsNeeded(dir);
          if (turns == 0) {
            state_ = ExplorerState::MovingForward;
            motion_->startFastForward();
          } else if (turns == 1) {
            state_ = ExplorerState::TurningToNextCell;
            motion_->turnRight();
          } else if (turns == 2) {
            // 180° turn: use two turnLeft() calls via state machine
            state_ = ExplorerState::TurningToNextCell2;
            motion_->turnLeft(); // First 90°
          } else {
            state_ = ExplorerState::TurningToNextCell;
            motion_->turnLeft();
          }
          found = true;
          break;
        }
      }

      if (!found) {
        // Check if all 144 cells visited - stop immediately without
        // backtracking
        int visitedCount = navigator_->getMap().countVisited();
        if (visitedCount >= 144) {
          state_ =
              ExplorerState::CalculatingPath; // Start destination navigation
          std::cout << "=== EXPLORATION COMPLETE (all 144 cells) ==="
                    << std::endl;
          std::cout << "Final position: (" << currentX_ << "," << currentY_
                    << ")" << std::endl;
          std::cout << "Visited: " << visitedCount << " cells" << std::endl;
          // Don't return true - continue to calculate path
        } else {
          // Not all cells visited, need to backtrack to find more
          state_ = ExplorerState::Backtracking;
        }
      }
      break;
    }

    // ----- TURNING TO NEXT CELL -----
    case ExplorerState::TurningToNextCell:
      currentHeading_ = nextMoveDirection_;
      state_ = ExplorerState::MovingForward;
      motion_->startFastForward();
      break;

    // ----- TURNING TO NEXT CELL (second 90° for 180° turns) -----
    case ExplorerState::TurningToNextCell2:
      // First turnLeft complete, do second turnLeft
      motion_->turnLeft(); // Second 90°
      state_ = ExplorerState::TurningToNextCell;
      break;

    // ----- MOVING FORWARD -----
    case ExplorerState::MovingForward:
      // Handled by isFastDriveActive check above
      break;

    // ----- BACKTRACKING -----
    case ExplorerState::Backtracking: {
      // Safety check: if all 144 cells visited, complete immediately
      int visitedCount = navigator_->getMap().countVisited();
      if (visitedCount >= 144) {
        state_ = ExplorerState::CalculatingPath; // Start destination navigation
        std::cout << "=== EXPLORATION COMPLETE (all 144 cells) ==="
                  << std::endl;
        std::cout << "Final position: (" << currentX_ << "," << currentY_ << ")"
                  << std::endl;
        std::cout << "Visited: " << visitedCount << " cells" << std::endl;
        // Don't return true - continue to calculate path
        break;
      }

      if (explorationStack_.empty()) {
        state_ = ExplorerState::CalculatingPath; // Partial maze, still navigate
        std::cout << "=== EXPLORATION COMPLETE (partial maze) ===" << std::endl;
        std::cout << "Final position: (" << currentX_ << "," << currentY_ << ")"
                  << std::endl;
        std::cout << "Visited: " << visitedCount << " cells" << std::endl;
        // Don't return true - continue to calculate path
        break;
      }

      auto prevCell = explorationStack_.top();
      explorationStack_.pop();

      if (prevCell.x == currentX_ && prevCell.y == currentY_) {
        // Same cell, try another direction
        state_ = ExplorerState::FindingNextCell;
        return false;
      }

      // Calculate direction to previous cell
      int dx = prevCell.x - currentX_;
      int dy = prevCell.y - currentY_;

      Heading dirBack;
      if (dy == 1)
        dirBack = Heading::NORTH;
      else if (dy == -1)
        dirBack = Heading::SOUTH;
      else if (dx == 1)
        dirBack = Heading::WEST;
      else
        dirBack = Heading::EAST;

      nextMoveDirection_ = dirBack;
      explorationStack_.push(prevCell); // Re-push for retry
      isBacktracking_ = true;           // Skip checkpoint detection on arrival

      int turns = turnsNeeded(dirBack);
      if (turns == 0) {
        state_ = ExplorerState::MovingForward;
        motion_->startFastForward();
      } else if (turns == 1) {
        state_ = ExplorerState::TurningToNextCell;
        motion_->turnRight();
      } else if (turns == 2) {
        // 180° turn: use two turnLeft() calls via state machine
        state_ = ExplorerState::TurningToNextCell2;
        motion_->turnLeft(); // First 90°
      } else {
        state_ = ExplorerState::TurningToNextCell;
        motion_->turnLeft();
      }
      break;
    }

    // ----- COMPLETE (transition to path calculation) -----
    case ExplorerState::Complete:
      // This state is now just a marker, actual work done in CalculatingPath
      state_ = ExplorerState::CalculatingPath;
      break;

    // ----- CALCULATING PATH -----
    case ExplorerState::CalculatingPath: {
      destination_ = navigator_->getDestination();
      Navigation::Vec2<int> start = {currentX_, currentY_};

      // Log A* start
      navigator_->logAStarStart(currentX_, currentY_, destination_.x,
                                destination_.y, robot_->getTime());
      navigator_->logCheckpointSummary();

      // Check if already at destination
      if (currentX_ == destination_.x && currentY_ == destination_.y) {
        navigator_->logAlreadyAtDestination(currentX_, currentY_,
                                            robot_->getTime());
        navigator_->flushLog();
        std::cout << "\n=== ALREADY AT DESTINATION! ===" << std::endl;
        state_ = ExplorerState::DestinationReached;
        return true;
      }

      // Find path
      auto path = navigator_->findPath(start, destination_);

      if (path.empty()) {
        std::cout << "ERROR: No path found to destination!" << std::endl;
        navigator_->logMessage("ERROR: No path found to destination");
        navigator_->flushLog();
        state_ = ExplorerState::DestinationReached;
        return true;
      }

      // Log path cells
      navigator_->logPath(path);

      // Convert to directions
      pathDirections_ = navigator_->pathToDirections(path);
      navigator_->logDirections(pathDirections_);
      navigator_->flushLog();

      // Console output
      std::cout << "\n=== NAVIGATING TO DESTINATION (" << destination_.x << ","
                << destination_.y << ") ===" << std::endl;
      std::cout << "Path: ";
      for (char d : pathDirections_)
        std::cout << d << " ";
      std::cout << "(" << pathDirections_.size() << " moves)" << std::endl;

      pathIndex_ = 0;
      state_ = ExplorerState::TurningToPath;
      break;
    }

    // ----- TURNING TO PATH -----
    case ExplorerState::TurningToPath: {
      if (pathIndex_ >= pathDirections_.size()) {
        // Shouldn't happen, but safety check
        state_ = ExplorerState::DestinationReached;
        return true;
      }

      char nextDirChar = pathDirections_[pathIndex_];
      Heading nextDir = charToHeading(nextDirChar);
      int turns = turnsNeeded(nextDir);

      if (turns == 0) {
        // Already facing correct direction - start forward immediately
        state_ = ExplorerState::MovingAlongPath;
        motion_->startFastForward();
      } else if (turns == 1) {
        // Turn right - will enter MovingAlongPath next cycle to start forward
        motion_->turnRight();
        currentHeading_ = nextDir;
        // Don't set state yet - turn is blocking, will complete this cycle
        state_ = ExplorerState::MovingAlongPath;
        motion_->startFastForward(); // Start forward after turn completes
      } else if (turns == 2) {
        // 180° turn: two left turns via TurningToPath2
        motion_->turnLeft();
        state_ = ExplorerState::TurningToPath2;
      } else { // turns == 3
        motion_->turnLeft();
        currentHeading_ = nextDir;
        state_ = ExplorerState::MovingAlongPath;
        motion_->startFastForward(); // Start forward after turn completes
      }
      break;
    }

    // ----- TURNING TO PATH 2 (second 90° for 180°) -----
    case ExplorerState::TurningToPath2: {
      char nextDirChar = pathDirections_[pathIndex_];
      Heading nextDir = charToHeading(nextDirChar);
      motion_->turnLeft();
      currentHeading_ = nextDir;
      state_ = ExplorerState::MovingAlongPath;
      motion_->startFastForward(); // Start forward after second turn
      break;
    }

    // ----- MOVING ALONG PATH -----
    case ExplorerState::MovingAlongPath:
      // Handled by isFastDriveActive check at top of update()
      // When fast forward completes, position is updated there
      break;

    // ----- DESTINATION REACHED -----
    case ExplorerState::DestinationReached:
      return true;

    default:
      break;
    }

    return false;
  }

  // -------------------------------------------------------------------------
  // STATUS QUERIES
  // -------------------------------------------------------------------------
  bool isComplete() const { return state_ == ExplorerState::Complete; }
  bool isDestinationReached() const {
    return state_ == ExplorerState::DestinationReached;
  }
  bool isRunning() const {
    return state_ != ExplorerState::Idle && state_ != ExplorerState::Complete &&
           state_ != ExplorerState::DestinationReached;
  }
  int getCurrentX() const { return currentX_; }
  int getCurrentY() const { return currentY_; }
  Navigation::Vec2<int> getDestination() const { return destination_; }

  const char *getHeadingStr() const {
    switch (currentHeading_) {
    case Heading::NORTH:
      return "NORTH";
    case Heading::EAST:
      return "EAST";
    case Heading::SOUTH:
      return "SOUTH";
    case Heading::WEST:
      return "WEST";
    }
    return "?";
  }

  const char *getStateStr() const {
    switch (state_) {
    case ExplorerState::Idle:
      return "Idle";
    case ExplorerState::MovingForward:
      return "MovingForward";
    case ExplorerState::LoggingCell:
      return "LoggingCell";
    case ExplorerState::CheckingFrontGreen:
      return "CheckingFrontGreen";
    case ExplorerState::TurningToLeft:
      return "TurningToLeft";
    case ExplorerState::CheckingLeftGreen:
      return "CheckingLeftGreen";
    case ExplorerState::TurningBackFromLeft:
      return "TurningBackFromLeft";
    case ExplorerState::TurningToRight1:
      return "TurningToRight1";
    case ExplorerState::TurningToRight2:
      return "TurningToRight2";
    case ExplorerState::CheckingRightGreen:
      return "CheckingRightGreen";
    case ExplorerState::TurningBackFromRight:
      return "TurningBackFromRight";
    case ExplorerState::FindingNextCell:
      return "FindingNextCell";
    case ExplorerState::TurningToNextCell:
      return "TurningToNextCell";
    case ExplorerState::TurningToNextCell2:
      return "TurningToNextCell2";
    case ExplorerState::Backtracking:
      return "Backtracking";
    case ExplorerState::Complete:
      return "Complete";
    case ExplorerState::CalculatingPath:
      return "CalculatingPath";
    case ExplorerState::TurningToPath:
      return "TurningToPath";
    case ExplorerState::TurningToPath2:
      return "TurningToPath2";
    case ExplorerState::MovingAlongPath:
      return "MovingAlongPath";
    case ExplorerState::DestinationReached:
      return "DestinationReached";
    }
    return "?";
  }
};

} // namespace Exploration

#endif // EXPLORER_H
