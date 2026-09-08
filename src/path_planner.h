#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "firmware_config.h"

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

    // 生成点到点直线路径 (当前点 -> 目标点), 返回路径点数 (2 或 0)
    int generateGoto(float x0, float y0, float x1, float y1);

    const Waypoint* getPath() const { return path_; }
    int getCount() const { return count_; }

private:
    Waypoint path_[MAX_WAYPOINTS];
    int count_ = 0;
};

#endif
