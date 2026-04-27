#pragma once
#include <vector>
#include <utility>

namespace multiraceline {

struct DetectedObstacle {
    double x;
    double y;
    double width;    // estimated lateral extent (m)
    double confidence;
};

class OpponentDetector {
public:
    struct Params {
        double max_range = 5.0;
        double cluster_gap = 0.3;        // max gap between points in same cluster (m)
        int min_cluster_points = 3;
        double max_obstacle_width = 0.6; // reject wider objects (walls)
        double min_obstacle_width = 0.05;
    };

    explicit OpponentDetector(const Params& params);

    std::vector<DetectedObstacle> detect(
        const std::vector<float>& ranges,
        float angle_min,
        float angle_increment,
        double car_x,
        double car_y,
        double car_yaw) const;

private:
    Params params_;

    struct Cluster {
        std::vector<std::pair<double, double>> points;
    };

    std::vector<Cluster> cluster(
        const std::vector<std::pair<double, double>>& pts) const;
};

}  // namespace multiraceline
