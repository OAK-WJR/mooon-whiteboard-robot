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
    // Only search forward, never back
    for (int i = current_wp_; i < count_; i++) {
        float d = distTo(rx, ry, path_[i].x, path_[i].y);
        if (d < best_dist) {
            best_dist = d;
            best = i;
        }
        // Stop searching early once the distance starts to grow
        if (d > best_dist + 0.05f) break;
    }
    return best;
}

void TrajectoryFollower::getLookAheadPoint(float rx, float ry, float& lx, float& ly) const {
    // Starting from the current nearest point, advance look_ahead_ along the path
    float remaining = look_ahead_;
    int idx = findClosestSegment(rx, ry);

    // Update the current waypoint (non-const needs mutable or an outside update)
    // First locate the look-ahead point
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

    // Past the end of the path, return the last point
    lx = path_[count_ - 1].x;
    ly = path_[count_ - 1].y;
}

bool TrajectoryFollower::compute(float robot_x, float robot_y, float robot_theta,
                                  BodyVelocity& out_vel) {
    if (!path_ || count_ == 0 || finished_) {
        out_vel = {0, 0, 0};
        return false;
    }

    // Update the current waypoint
    while (current_wp_ < count_ - 1 &&
           distTo(robot_x, robot_y, path_[current_wp_].x, path_[current_wp_].y) < goal_tol_) {
        current_wp_++;
    }

    // Reached the end point
    if (current_wp_ >= count_ - 1 &&
        distTo(robot_x, robot_y, path_[count_ - 1].x, path_[count_ - 1].y) < goal_tol_) {
        out_vel = {0, 0, 0};
        finished_ = true;
        return false;
    }

    // Look-ahead target point
    float lx, ly;
    getLookAheadPoint(robot_x, robot_y, lx, ly);

    // Compute the error (in the body frame)
    float dx = lx - robot_x;
    float dy = ly - robot_y;
    float cos_th = cosf(robot_theta);
    float sin_th = sinf(robot_theta);

    // World-frame error → body-frame error
    float ex = cos_th * dx + sin_th * dy;   // forward
    float ey = -sin_th * dx + cos_th * dy;  // left

    // Final slowdown: the closer to the target, the lower the speed, to avoid overshooting and circling
    float dist_to_goal = distTo(robot_x, robot_y, path_[count_ - 1].x, path_[count_ - 1].y);
    float v_cap = max_speed_;
    if (dist_to_goal < END_DECEL_DIST)
        v_cap = fmaxf(MIN_APPROACH_SPEED, max_speed_ * dist_to_goal / END_DECEL_DIST);

    // Proportional control (capped by the final slowdown)
    float dist = sqrtf(ex * ex + ey * ey);
    float scale = (dist > 0.001f) ? fminf(kp_linear_ * dist, v_cap) / dist : 0;

    out_vel.vx = ex * scale;
    out_vel.vy = ey * scale;

    // Heading hold: actively pull theta back to 0 to cancel rotational drift -> drive straight
    float theta_err = -robot_theta;
    while (theta_err >  (float)M_PI) theta_err -= 2.0f * (float)M_PI;
    while (theta_err < -(float)M_PI) theta_err += 2.0f * (float)M_PI;
    out_vel.omega = fmaxf(-MAX_ANGULAR_SPEED,
                          fminf(MAX_ANGULAR_SPEED, KP_ANGULAR * theta_err));

    return true;
}
