#ifndef TRAJECTORY_FOLLOWER_H
#define TRAJECTORY_FOLLOWER_H

#include "path_planner.h"
#include "kinematics.h"
#include "firmware_config.h"
#include <math.h>

class TrajectoryFollower {
public:
    void setPath(const Waypoint* path, int count);
    void setSpeed(float max_speed) { max_speed_ = max_speed; }

    // 运行时可调参数 (串口调参用), 默认取 config.h 中的值
    void setLookAhead(float d)      { look_ahead_ = d; }
    void setGoalTolerance(float d)  { goal_tol_ = d; }
    void setLinearGain(float kp)    { kp_linear_ = kp; }
    float getLookAhead() const      { return look_ahead_; }
    float getGoalTolerance() const  { return goal_tol_; }
    float getLinearGain() const     { return kp_linear_; }
    float getMaxSpeed() const       { return max_speed_; }

    // 根据当前位置计算目标机体速度
    // 返回 true 如果路径还没走完
    bool compute(float robot_x, float robot_y, float robot_theta,
                 BodyVelocity& out_vel);

    int getCurrentWaypoint() const { return current_wp_; }
    int getTotalWaypoints() const { return count_; }
    float getProgress() const { return count_ > 0 ? (float)current_wp_ / count_ : 0; }
    bool isFinished() const { return finished_; }

    void reset();

private:
    const Waypoint* path_ = nullptr;
    int count_ = 0;
    int current_wp_ = 0;
    float max_speed_ = MAX_LINEAR_SPEED;
    float look_ahead_ = LOOK_AHEAD_DIST;
    float goal_tol_   = GOAL_TOLERANCE;
    float kp_linear_  = KP_LINEAR;
    bool finished_ = false;

    float distTo(float x1, float y1, float x2, float y2) const;
    int findClosestSegment(float rx, float ry) const;
    void getLookAheadPoint(float rx, float ry, float& lx, float& ly) const;
};

#endif
