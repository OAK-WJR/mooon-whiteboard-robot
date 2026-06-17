#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "config.h"

struct Waypoint {
    float x;
    float y;
};

class PathPlanner {
public:
    // 生成 zigzag 覆盖路径
    // area: 左下角 (ax,ay), 宽 w, 高 h, 行间距 spacing
    // 返回路径点数
    int generateZigzag(float ax, float ay, float w, float h, float spacing);

    const Waypoint* getPath() const { return path_; }
    int getCount() const { return count_; }

private:
    Waypoint path_[MAX_WAYPOINTS];
    int count_ = 0;
};

#endif
