#pragma once
#include <vector>
#include <array>
#include <cmath>

namespace multiraceline {

struct FrenetPoint {
    double s;
    double d;
};

class FrenetFrame {
public:
    FrenetFrame() = default;
    explicit FrenetFrame(const std::vector<std::array<double, 2>>& centerline);

    FrenetPoint cartesianToFrenet(double x, double y) const;
    std::pair<double, double> frenetToCartesian(double s, double d) const;

    double totalLength() const { return total_length_; }
    bool isInitialized() const { return initialized_; }

private:
    std::vector<std::array<double, 2>> centerline_;
    std::vector<double> s_values_;
    std::vector<double> headings_;
    double total_length_ = 0.0;
    bool initialized_ = false;
    int N_ = 0;
};

}  // namespace multiraceline
