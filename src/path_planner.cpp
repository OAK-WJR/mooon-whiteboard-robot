#include "path_planner.h"

int PathPlanner::generateZigzag(float ax, float ay, float w, float h, float spacing) {
    count_ = 0;
    if (w <= 0 || h <= 0 || spacing <= 0) return 0;

    int lines = (int)(h / spacing) + 1;
    bool left_to_right = true;

    for (int i = 0; i < lines && count_ < MAX_WAYPOINTS - 1; i++) {
        float y = ay + i * spacing;
        if (y > ay + h) y = ay + h;

        if (left_to_right) {
            path_[count_++] = {ax, y};
            path_[count_++] = {ax + w, y};
        } else {
            path_[count_++] = {ax + w, y};
            path_[count_++] = {ax, y};
        }
        left_to_right = !left_to_right;
    }
    return count_;
}

int PathPlanner::generateGoto(float x0, float y0, float x1, float y1) {
    count_ = 0;
    // No path if the start and end points are the same
    float dx = x1 - x0, dy = y1 - y0;
    if (dx * dx + dy * dy < 1e-6f) return 0;
    path_[count_++] = {x0, y0};
    path_[count_++] = {x1, y1};
    return count_;
}
