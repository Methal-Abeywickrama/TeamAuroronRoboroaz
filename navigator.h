#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <string>
#include <vector>
#include <iostream>
#include <random>
#include <cmath>
#include <algorithm> // For std::abs

namespace Navigation {

    // --- Constants ---
    #ifndef M_PI
        constexpr double PI = 3.1415926535;
    #else
        constexpr double PI = M_PI;
    #endif

    // --- 1. Vec2 Struct (Template Version from Logic Source) ---
    template <typename T>
    struct Vec2 {
        T x, y;
        Vec2() : x(0), y(0) {}
        Vec2(T x_val, T y_val) : x(x_val), y(y_val) {}

        Vec2 operator-(const Vec2& other) const {
            return {x - other.x, y - other.y};
        }
        bool operator!=(const Vec2& other) const {
            return !(*this == other);
        }
        bool operator==(const Vec2& other) const {
            return x == other.x && y == other.y;
        }
    };

    // --- 2. Odometry Class (Required for Map::head logic) ---
    class Odometry {
    private:
        double radius;
        double axleLength;
        double x;
        double y;
        double theta;
        double CELL_SIZE;

    public:
        Odometry(double r, double l, double s) : radius(r), axleLength(l), x(0.0), y(0.0), theta(0.0), CELL_SIZE(s) {}

        void update(double v_left_rad, double v_right_rad, double dt) {
            double v_left = radius * v_left_rad;
            double v_right = radius * v_right_rad;

            double v = (v_left + v_right) / 2;
            double w = (v_left - v_right) / axleLength;

            x += v * std::cos(theta) * dt;
            y += v * std::sin(theta) * dt;
            theta += w * dt;

            if (theta > PI) { theta -= 2 * PI; }
            if (theta <= -PI) { theta += 2 * PI; }
        }

        Vec2<double> getPos() const {
            return {x, y};
        }

        int getGrid(double z) const {
            int coord = static_cast<int>(std::round(z / CELL_SIZE));
            return coord;
        }

        double getX() const { return x; }
        double getY() const { return y; }
        double getAngle() const { return theta; }
    };

    // --- 3. Map Class (Logic from Source 1) ---
    class Map {
    private:
        int column;
        int row;
        Vec2<int> origin;
        double cell_size;
        int X_check, Y_check;
        std::vector<int> map;
        std::vector<Vec2<int>> checkpoints;

    public:
        // Default constructor
        Map() : row(0), column(0), origin({0, 0}), cell_size(0), X_check(0), Y_check(0) {}

        Map(int r, int c, Vec2<int> o, double size) {
            column = c;
            row = r;
            origin = o;
            cell_size = size;
            X_check = 0;
            Y_check = 0;
            map.resize(r * c, 0);
        }

        int index(int x, int y) const {
            return (y * column + x);
        }

        // Logic from Source 1: using bool array
        void update_cell(bool data[4], int coord_x, int coord_y) {
            if (!isValid(coord_x, coord_y)) return;
            int i = index(coord_x, coord_y);
            int cell_num = 1 * data[0] + 2 * data[1] + 4 * data[2] + 8 * data[3];

            if (!map[i]) { map[i] = cell_num; }
        }

        // Logic from Source 1: Odometry integration
        void head(bool north_rb, bool east_rb, bool south_rb, bool west_rb, const Odometry& odem, int coord_x, int coord_y) {
            int i = index(coord_x, coord_y);
            if (!isValid(coord_x, coord_y)) return;
            
            if (map[i]) { return; }
            
            bool sensorData[4] = {west_rb, north_rb, east_rb, south_rb};
            bool updateData[4];
            
            int headI = static_cast<int>(std::round(odem.getAngle() / (PI / 2))) + 1;
            
            for (int k = 0; k < 4; k++) { 
                updateData[k] = sensorData[headI];
                headI += 1;
                headI = headI % 4;
            }
            update_cell(updateData, coord_x, coord_y); 
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

        Vec2<int> getDim() const {
            return {row, column};
        }

        int getValue(int x, int y) const {
            if (!isValid(x, y)) return 0;
            int ind = index(x, y);
            return map[ind];
        }

        bool isValid(int x, int y) const {
            return (y >= 0 && y < row && x >= 0 && x < column);
        }

        void updateCheckpointCount(int x, int y) {
            Vec2<int> point = {x, y};
            for (const auto& visited : checkpoints) {
                if (visited == point) {
                    return; // Already visited
                }
            }
            checkpoints.push_back(point);
            X_check += x;
            Y_check += y;
        }

        Vec2<int> getDest() const {
            int dest_x = X_check % 12; 
            int dest_y = Y_check % 12;
            return {dest_x, dest_y};
        }
    };

    // --- 4. Helper Structures & Functions ---

    struct Cell {
        int parent_i, parent_j;
        double f, g, h;
    };

    // Global helper for random numbers
    inline int getRandom(int min, int max) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<> distr(min, max);
        return distr(gen);
    }

    // --- 5. Navigator Class (Structure from Source 2) ---
    class Navigator {
    private:
        Map currentMap;

    public:
        Navigator() {}

        Navigator(Map m) : currentMap(m) {}

        ~Navigator() {}

        void setMap(Map m) {
            currentMap = m;
        }

        // Accessor to modify the map (for Odometry updates)
        Map& getMap() {
            return currentMap;
        }

        void debugPrintMap() {
            currentMap.printMap();
        }

        bool isValid(int x, int y) {
            return currentMap.isValid(x, y);
        }

        // Integrated isUnblocked logic as a member function
        bool isUnblocked(Vec2<int> curr, Vec2<int> dest) {
            Vec2<int> direction = dest - curr;

            const int rowBit[] = {8, 0, 2};
            const int colBit[] = {1, 0, 4};

            if (std::abs(direction.x) > 1 || std::abs(direction.y) > 1) return false;

            // Safety check for direction array indices
            int xIndex = direction.x + 1;
            int yIndex = direction.y + 1;
            if (xIndex < 0 || xIndex > 2 || yIndex < 0 || yIndex > 2) return false;

            int checkBit = rowBit[xIndex] + colBit[yIndex];
            int mapBit = currentMap.getValue(curr.x, curr.y);

            if (checkBit & mapBit) {
                return false;
            }
            return true;
        }

    };

} // namespace Navigation

#endif // NAVIGATOR_H