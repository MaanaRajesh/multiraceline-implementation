#include "multiraceline_racing/opponent_detector.hpp"
#include <cmath>
#include <limits>

namespace multiraceline {

OpponentDetector::OpponentDetector(const Params& params) : params_(params) {}

std::vector<DetectedObstacle> OpponentDetector::detect(
    const std::vector<float>& ranges,
    float angle_min,
    float angle_increment,
    double car_x,
    double car_y,
    double car_yaw) const
{
    // Convert valid scan points to world-frame (x, y)
    std::vector<std::pair<double, double>> pts;
    pts.reserve(ranges.size());

    for (size_t i = 0; i < ranges.size(); ++i) {
        float r = ranges[i];
        if (!std::isfinite(r) || r < 0.05f || r > static_cast<float>(params_.max_range)) {
            continue;
        }
        double angle = car_yaw + angle_min + i * angle_increment;
        pts.push_back({
            car_x + r * std::cos(angle),
            car_y + r * std::sin(angle)
        });
    }

    auto clusters = cluster(pts);

    std::vector<DetectedObstacle> obstacles;
    for (const auto& c : clusters) {
        if (static_cast<int>(c.points.size()) < params_.min_cluster_points) continue;

        // Centroid
        double cx = 0.0, cy = 0.0;
        double xmin = std::numeric_limits<double>::max();
        double xmax = -std::numeric_limits<double>::max();
        double ymin = xmin, ymax = xmax;
        for (const auto& p : c.points) {
            cx += p.first; cy += p.second;
            xmin = std::min(xmin, p.first); xmax = std::max(xmax, p.first);
            ymin = std::min(ymin, p.second); ymax = std::max(ymax, p.second);
        }
        cx /= c.points.size();
        cy /= c.points.size();

        double width = std::hypot(xmax - xmin, ymax - ymin);
        if (width < params_.min_obstacle_width || width > params_.max_obstacle_width) continue;

        double dist = std::hypot(cx - car_x, cy - car_y);
        double confidence = 1.0 - dist / params_.max_range;

        obstacles.push_back({cx, cy, width, confidence});
    }

    return obstacles;
}

std::vector<OpponentDetector::Cluster> OpponentDetector::cluster(
    const std::vector<std::pair<double, double>>& pts) const
{
    std::vector<Cluster> clusters;
    if (pts.empty()) return clusters;

    Cluster current;
    current.points.push_back(pts[0]);

    for (size_t i = 1; i < pts.size(); ++i) {
        double dx = pts[i].first - pts[i-1].first;
        double dy = pts[i].second - pts[i-1].second;
        if (std::hypot(dx, dy) > params_.cluster_gap) {
            clusters.push_back(std::move(current));
            current = Cluster{};
        }
        current.points.push_back(pts[i]);
    }
    clusters.push_back(std::move(current));

    return clusters;
}

}  // namespace multiraceline
