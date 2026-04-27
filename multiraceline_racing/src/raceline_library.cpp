#include "multiraceline_racing/raceline_library.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace multiraceline {

const std::string RacelineLibrary::OPTIMAL       = "optimal";
const std::string RacelineLibrary::INSIDE_ATTACK  = "inside_attack";
const std::string RacelineLibrary::OUTSIDE_ATTACK = "outside_attack";
const std::string RacelineLibrary::DEFENSIVE      = "defensive";

static Waypoints loadCSV(const std::string& path)
{
    Waypoints wps;
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot open raceline CSV: " + path);
    }
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string tok;
        Waypoint wp{};
        int col = 0;
        while (std::getline(ss, tok, ',') && col < 4) {
            wp[col++] = std::stod(tok);
        }
        if (col >= 2) wps.push_back(wp);
    }
    return wps;
}

bool RacelineLibrary::loadCenterline(const std::string& csv_path)
{
    try {
        auto wps = loadCSV(csv_path);
        std::vector<std::array<double, 2>> cl;
        cl.reserve(wps.size());
        for (const auto& wp : wps) {
            cl.push_back({wp[0], wp[1]});
        }
        frenet_ = FrenetFrame(cl);
        return true;
    } catch (...) {
        return false;
    }
}

bool RacelineLibrary::loadRaceline(const std::string& name, const std::string& csv_path)
{
    try {
        racelines_[name] = loadCSV(csv_path);
        return true;
    } catch (...) {
        return false;
    }
}

const Waypoints& RacelineLibrary::getRaceline(const std::string& name) const
{
    auto it = racelines_.find(name);
    return (it != racelines_.end()) ? it->second : empty_;
}

bool RacelineLibrary::hasRaceline(const std::string& name) const
{
    return racelines_.count(name) > 0;
}

}  // namespace multiraceline
