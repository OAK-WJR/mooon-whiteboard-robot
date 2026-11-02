#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "firmware_config.h"

struct Waypoint {
    float x;
    float y;
};

class PathPlanner {
public:
    // Generate a zigzag coverage path
    // area: bottom-left corner (ax,ay), width w, height h, row spacing spacing
    // Returns the number of path points
    int generateZigzag(float ax, float ay, float w, float h, float spacing);

    // Generate a straight point-to-point path (current point -> target point); returns the number of path points (2 or 0)
    int generateGoto(float x0, float y0, float x1, float y1);

    const Waypoint* getPath() const { return path_; }
    int getCount() const { return count_; }

private:
    Waypoint path_[MAX_WAYPOINTS];
    int count_ = 0;
};

#endif
