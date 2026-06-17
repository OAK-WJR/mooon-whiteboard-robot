#include "trajectory_follower.h"

void TrajectoryFollower::setPath(const Waypoint* path, int count) {
    path_ = path;
    count_ = count;
    current_wp_ = 0;
    finished_ = false;
}

void TrajectoryFollower::reset() {
    path_ = nullptr;
    count_ = 0;
    current_wp_ = 0;
    finished_ = false;
}

float TrajectoryFollower::distTo(float x1, float y1, float x2, float y2) const {
    float dx = x2 - x1, dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

int TrajectoryFollower::findClosestSegment(float rx, float ry) const {
    int best = current_wp_;
    float best_dist = 1e9f;
    // 只向前搜索，不回头
    for (int i = current_wp_; i < count_; i++) {
        float d = distTo(rx, ry, path_[i].x, path_[i].y);
        if (d < best_dist) {
            best_dist = d;
            best = i;
        }
        // 如果距离开始增大，提前停止搜索
        if (d > best_dist + 0.05f) break;
    }
    return best;
}

void TrajectoryFollower::getLookAheadPoint(float rx, float ry, float& lx, float& ly) const {
    // 从当前最近点开始，沿路径前进 look_ahead_
    float remaining = look_ahead_;
    int idx = findClosestSegment(rx, ry);

    // 更新 current waypoint（非 const 需要 mutable 或者外部更新）
    // 先定位到 look-ahead 点
    float cx = path_[idx].x, cy = path_[idx].y;

    for (int i = idx; i < count_ - 1 && remaining > 0; i++) {
        float seg_len = distTo(path_[i].x, path_[i].y, path_[i + 1].x, path_[i + 1].y);
        if (seg_len < 0.0001f) continue;

        if (remaining <= seg_len) {
            float t = remaining / seg_len;
            lx = path_[i].x + t * (path_[i + 1].x - path_[i].x);
            ly = path_[i].y + t * (path_[i + 1].y - path_[i].y);
            return;
        }
        remaining -= seg_len;
    }

    // 超过路径末端，返回最后一个点
    lx = path_[count_ - 1].x;
    ly = path_[count_ - 1].y;
}

bool TrajectoryFollower::compute(float robot_x, float robot_y, float robot_theta,
                                  BodyVelocity& out_vel) {
    if (!path_ || count_ == 0 || finished_) {
        out_vel = {0, 0, 0};
        return false;
    }

    // 更新当前 waypoint
    while (current_wp_ < count_ - 1 &&
           distTo(robot_x, robot_y, path_[current_wp_].x, path_[current_wp_].y) < goal_tol_) {
        current_wp_++;
    }

    // 到达终点
    if (current_wp_ >= count_ - 1 &&
        distTo(robot_x, robot_y, path_[count_ - 1].x, path_[count_ - 1].y) < goal_tol_) {
        out_vel = {0, 0, 0};
        finished_ = true;
        return false;
    }

    // Look-ahead 目标点
    float lx, ly;
    getLookAheadPoint(robot_x, robot_y, lx, ly);

    // 计算误差 (在机体坐标系下)
    float dx = lx - robot_x;
    float dy = ly - robot_y;
    float cos_th = cosf(robot_theta);
    float sin_th = sinf(robot_theta);

    // 世界坐标误差 → 机体坐标误差
    float ex = cos_th * dx + sin_th * dy;   // 前方
    float ey = -sin_th * dx + cos_th * dy;  // 左方

    // 比例控制
    float dist = sqrtf(ex * ex + ey * ey);
    float scale = (dist > 0.001f) ? fminf(kp_linear_ * dist, max_speed_) / dist : 0;

    out_vel.vx = ex * scale;
    out_vel.vy = ey * scale;
    out_vel.omega = 0;  // 全向轮不需要转向，直接侧移

    return true;
}
