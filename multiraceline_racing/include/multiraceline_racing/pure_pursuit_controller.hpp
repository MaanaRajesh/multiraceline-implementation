#pragma once
#include <vector>
#include <array>

namespace multiraceline {

using Waypoint = std::array<double, 4>;  // x, y, yaw, speed

struct DriveCommand {
    double steering_angle;
    double speed;
};

class PurePursuitController {
public:
    struct Params {
        double lookahead_min = 0.5;
        double lookahead_max = 2.0;
        double fast_speed = 2.0;
        double slow_speed = 0.5;
        double steering_limit = 0.4189;
        double steer_alpha = 0.8;
        bool use_waypoint_speed = true;
        int lookahead_window = 20;
    };

    explicit PurePursuitController(const Params& params);

    DriveCommand compute(
        double car_x, double car_y, double car_yaw,
        const std::vector<Waypoint>& waypoints);

    void resetIndex() { current_idx_ = 0; }

private:
    Params params_;
    int current_idx_ = 0;
    double prev_steering_ = 0.0;

    int findLookaheadWaypoint(
        double x, double y,
        const std::vector<Waypoint>& waypoints,
        double lookahead);

    double adaptiveLookahead(
        double x, double y,
        const std::vector<Waypoint>& waypoints);
};

}  // namespace multiraceline
