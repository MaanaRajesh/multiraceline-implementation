#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include <array>
#include "multiraceline_racing/frenet_frame.hpp"

namespace multiraceline {

// [x, y, yaw, speed] — matches pure_pursuit_node waypoint format
using Waypoint = std::array<double, 4>;
using Waypoints = std::vector<Waypoint>;

class RacelineLibrary {
public:
    static const std::string OPTIMAL;
    static const std::string INSIDE_ATTACK;
    static const std::string OUTSIDE_ATTACK;
    static const std::string DEFENSIVE;

    RacelineLibrary() = default;

    bool loadCenterline(const std::string& csv_path);
    bool loadRaceline(const std::string& name, const std::string& csv_path);

    const Waypoints& getRaceline(const std::string& name) const;
    bool hasRaceline(const std::string& name) const;

    const FrenetFrame& frenetFrame() const { return frenet_; }

private:
    std::unordered_map<std::string, Waypoints> racelines_;
    FrenetFrame frenet_;
    Waypoints empty_;
};

}  // namespace multiraceline
