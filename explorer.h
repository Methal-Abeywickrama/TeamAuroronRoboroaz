#ifndef EXPLORER_H
#define EXPLORER_H

#include "navigator.h"
#include "motion_control.h"
#include "sensing.h"
#include <stack>   

namespace Navigation {

class Explorer {

private:
Map &map;
Navigator &nav;
Odometry &odo;

public:

Explorer(Map &m, Navigator &n, Odometry &o)
    : map(m), nav(n), odo(o) {}

// ----- Robot move wrapper -----
void move(int dir) {
    switch (dir) {
        case 0:{
            moveForward()
    }
        case 1:{
            turnRight()
            moveForward()
    }
        case 2:{
            rotate(3.1415)
            moveForward()
    }
        case 3:{
            turnLeft()
            moveForward()
    }
}

// ---------- DFS explore ----------
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

        // ------- Explore all 4 directions -------
        for (int dir = 0; dir < 4; dir++) {

            Vec2<int> next(cx + dx[dir], cy + dy[dir]);

            if (!nav.isValid(next.x, next.y)) continue;
            if (!nav.isUnblocked(cur, next)) continue;

            // If already mapped, skip
            if (nav.getMap().getValue(next.x, next.y) != 0) continue;

            move(dir);
            st.push(next);
            moved = true;
            break;
        }

        // --------- BACKTRACK ----------
        if (!moved) {
            st.pop();

            if (!st.empty()) {
                Vec2<int> prev = st.top();
