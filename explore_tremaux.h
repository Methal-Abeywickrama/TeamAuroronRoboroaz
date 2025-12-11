#ifndef EXPLORER_TREMAUX_H
#define EXPLORER_TREMAUX_H

#include "navigator.h"
#include "motion_control.h"
#include "sensing.h"
#include <stack>   
#include <array>

namespace Navigation {

class Explorer {
private:
    Map &map;
    Navigator &nav;
    Odometry &odo;

    // Visit count for each cell per direction (0=N,1=E,2=S,3=W)
    std::vector<std::vector<std::array<int, 4>>> visit;

public:
    Explorer(Map &m, Navigator &n, Odometry &o)
        : map(m), nav(n), odo(o) 
    {
        Vec2<int> dim = map.getDim();
        visit.resize(dim.x, std::vector<std::array<int,4>>(dim.y, {0,0,0,0}));
    }

    // ----- Robot move wrapper -----
    void moveRobot(int dir) {
        switch (dir) {
            case 0: moveForward(); break;
            case 1: turnRight(); moveForward(); break;
            case 2: rotate(3.1415); moveForward(); break;
            case 3: turnLeft(); moveForward(); break;
        }
    }

    // ---------- Trémaux explore ----------
    void explore(Vec2<int> start) {

        std::stack<Vec2<int>> st;
        st.push(start);

        int dx[4] = {0, 1, 0, -1};   // N, E, S, W
        int dy[4] = {1, 0, -1, 0};

        while (!st.empty()) {
            Vec2<int> cur = st.top();
            int cx = cur.x;
            int cy = cur.y;

            // Unvisited → map update
            if (nav.getMap().getValue(cx, cy) == 0) {
                bool N = isWallAtFront();
                bool E = isWallAtRight();
                bool S = isWallAtBack();
                bool W = isWallAtLeft();

                nav.getMap().head_update(N, E, S, W, odo, cx, cy);
                nav.getMap().updateCheckpointCount(cx, cy);
            }

            bool moved = false;
            int nextDir = -1;

            // ---- Find next move according to Trémaux ----
            for (int dir = 0; dir < 4; dir++) {
                Vec2<int> next(cx + dx[dir], cy + dy[dir]);

                if (!nav.isValid(next.x, next.y)) continue;
                if (!nav.isUnblocked(cur, next)) continue;

                // Prefer paths not yet visited in this direction
                if (visit[cx][cy][dir] == 0) {
                    nextDir = dir;
                    break;
                }
            }

            // If no unvisited path, choose a path visited only once
            if (nextDir == -1) {
                for (int dir = 0; dir < 4; dir++) {
                    Vec2<int> next(cx + dx[dir], cy + dy[dir]);
                    if (!nav.isValid(next.x, next.y)) continue;
                    if (!nav.isUnblocked(cur, next)) continue;
                    if (visit[cx][cy][dir] == 1) {
                        nextDir = dir;
                        break;
                    }
                }
            }

            if (nextDir != -1) {
                Vec2<int> next(cx + dx[nextDir], cy + dy[nextDir]);
                
                // Mark path in both directions
                visit[cx][cy][nextDir]++;
                visit[next.x][next.y][(nextDir + 2) % 4]++;

                moveRobot(nextDir);
                st.push(next);
                moved = true;
            }

            // ---- Backtrack if stuck ----
            if (!moved) {
                st.pop();
                if (!st.empty()) {
                    Vec2<int> prev = st.top();
                    int backDir = -1;
                    for (int d = 0; d < 4; d++) {
                        if (prev.x - cx == dx[d] && prev.y - cy == dy[d]) {
                            backDir = d;
                            break;
                        }
                    }
                    if (backDir != -1)
                        moveRobot((backDir + 2) % 4);
                }
            }
        }
    }
};

} // namespace Navigation

#endif // EXPLORER_TREMAUX_H
