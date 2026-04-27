#include "multiraceline_racing/pure_pursuit_controller.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace multiraceline {

PurePursuitController::PurePursuitController(const Params& params) : params_(params) {}

double PurePursuitController::adaptiveLookahead(
    double x, double y,
    const std::vector<Waypoint>& waypoints)
{
    int n = static_cast<int>(waypoints.size());
    if (n == 0) return params_.lookahead_min;

    // Look at curvature in the window ahead of current_idx_
    double max_curv = 0.0;
    for (int i = 1; i < params_.lookahead_window - 1 && i < n - 1; ++i) {
        int i0 = (current_idx_ + i - 1) % n;
        int i1 = (current_idx_ + i    ) % n;
        int i2 = (current_idx_ + i + 1) % n;

        double d1x = waypoints[i1][0] - waypoints[i0][0];
        double d1y = waypoints[i1][1] - waypoints[i0][1];
        double d2x = waypoints[i2][0] - waypoints[i1][0];
        double d2y = waypoints[i2][1] - waypoints[i1][1];

        double cross = std::abs(d1x * d2y - d1y * d2x);
        double denom = std::hypot(d1x, d1y) * std::hypot(d2x, d2y) *
                       std::hypot(waypoints[i2][0] - waypoints[i0][0],
                                  waypoints[i2][1] - waypoints[i0][1]);
        double curv = (denom > 1e-6) ? cross / denom : 0.0;
        max_curv = std::max(max_curv, curv);
    }

    // High curvature → shorter lookahead
    double norm = std::min(max_curv / 1.0, 1.0);
    return params_.lookahead_max - norm * (params_.lookahead_max - params_.lookahead_min);
}

int PurePursuitController::findLookaheadWaypoint(
    double x, double y,
    const std::vector<Waypoint>& waypoints,
    double lookahead)
{
    int n = static_cast<int>(waypoints.size());
    if (n == 0) return -1;

    // Advance current_idx_ to closest point within forward search window
    int window = std::max(n / 4, 10);
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < window; ++i) {
        int idx = (current_idx_ + i) % n;
        double d = std::hypot(waypoints[idx][0] - x, waypoints[idx][1] - y);
        if (d < min_dist) { min_dist = d; current_idx_ = idx; }
    }

    // Find first waypoint >= lookahead distance ahead
    for (int i = 0; i < n; ++i) {
        int idx = (current_idx_ + i) % n;
        double d = std::hypot(waypoints[idx][0] - x, waypoints[idx][1] - y);
        if (d >= lookahead) return idx;
    }
    return current_idx_;
}

DriveCommand PurePursuitController::compute(
    double car_x, double car_y, double car_yaw,
    const std::vector<Waypoint>& waypoints)
{
    if (waypoints.empty()) return {0.0, 0.0};

    double lookahead = adaptiveLookahead(car_x, car_y, waypoints);
    int goal_idx = findLookaheadWaypoint(car_x, car_y, waypoints, lookahead);
    if (goal_idx < 0) return {0.0, 0.0};

    // Transform goal to vehicle frame
    double dx = waypoints[goal_idx][0] - car_x;
    double dy = waypoints[goal_idx][1] - car_y;
    double local_x =  dx * std::cos(car_yaw) + dy * std::sin(car_yaw);
    double local_y = -dx * std::sin(car_yaw) + dy * std::cos(car_yaw);

    // Pure pursuit curvature → steering angle
    double L = std::hypot(local_x, local_y);
    double raw_steering = 0.0;
    if (L > 1e-3) {
        raw_steering = std::atan(2.0 * local_y / (L * L));
    }
    raw_steering = std::clamp(raw_steering, -params_.steering_limit, params_.steering_limit);

    // EMA steering filter
    double steering = params_.steer_alpha * raw_steering + (1.0 - params_.steer_alpha) * prev_steering_;
    prev_steering_ = steering;

    // Speed
    double speed;
    if (params_.use_waypoint_speed && waypoints[goal_idx][3] > 0.0) {
        double ratio = std::clamp(waypoints[goal_idx][3], 0.0, 1.0);
        speed = params_.slow_speed + ratio * (params_.fast_speed - params_.slow_speed);
    } else {
        double norm = std::min(std::abs(steering) / params_.steering_limit, 1.0);
        speed = params_.fast_speed - norm * (params_.fast_speed - params_.slow_speed);
    }

    return {steering, speed};
}

}  // namespace multiraceline
