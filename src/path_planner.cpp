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
