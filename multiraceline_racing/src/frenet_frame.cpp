#include "multiraceline_racing/frenet_frame.hpp"
#include <algorithm>
#include <limits>
#include <cmath>

namespace multiraceline {

FrenetFrame::FrenetFrame(const std::vector<std::array<double, 2>>& centerline)
    : centerline_(centerline), N_(static_cast<int>(centerline.size()))
{
    if (N_ < 2) return;

    s_values_.resize(N_, 0.0);
    headings_.resize(N_, 0.0);

    for (int i = 1; i < N_; ++i) {
        double dx = centerline_[i][0] - centerline_[i-1][0];
        double dy = centerline_[i][1] - centerline_[i-1][1];
        s_values_[i] = s_values_[i-1] + std::hypot(dx, dy);
    }

    double dx_close = centerline_[0][0] - centerline_[N_-1][0];
    double dy_close = centerline_[0][1] - centerline_[N_-1][1];
    total_length_ = s_values_[N_-1] + std::hypot(dx_close, dy_close);

    for (int i = 0; i < N_ - 1; ++i) {
        headings_[i] = std::atan2(
            centerline_[i+1][1] - centerline_[i][1],
            centerline_[i+1][0] - centerline_[i][0]);
    }
    headings_[N_-1] = std::atan2(dy_close, dx_close);

    initialized_ = true;
}

FrenetPoint FrenetFrame::cartesianToFrenet(double x, double y) const
{
    if (!initialized_) return {0.0, 0.0};

    double best_dist = std::numeric_limits<double>::max();
    int best_idx = 0;
    for (int i = 0; i < N_; ++i) {
        double d = std::hypot(centerline_[i][0] - x, centerline_[i][1] - y);
        if (d < best_dist) { best_dist = d; best_idx = i; }
    }

    double best_s = s_values_[best_idx];
    double best_signed_d = best_dist;

    auto trySegment = [&](int i0, int i1) {
        i1 = i1 % N_;
        double segx = centerline_[i1][0] - centerline_[i0][0];
        double segy = centerline_[i1][1] - centerline_[i0][1];
        double seg_len_sq = segx * segx + segy * segy;
        if (seg_len_sq < 1e-18) return;

        double t = ((x - centerline_[i0][0]) * segx + (y - centerline_[i0][1]) * segy) / seg_len_sq;
        t = std::clamp(t, 0.0, 1.0);

        double projx = centerline_[i0][0] + t * segx;
        double projy = centerline_[i0][1] + t * segy;
        double dist = std::hypot(x - projx, y - projy);

        if (dist < std::abs(best_signed_d)) {
            double seg_len = std::sqrt(seg_len_sq);
            double s_seg = (i0 == N_ - 1)
                ? (total_length_ - s_values_[N_-1])
                : (s_values_[i1] - s_values_[i0]);
            best_s = s_values_[i0] + t * s_seg;

            // Positive d = left of direction of travel
            double cross = segx * (y - projy) - segy * (x - projx);
            best_signed_d = std::copysign(dist, cross);
            (void)seg_len;
        }
    };

    trySegment((best_idx - 1 + N_) % N_, best_idx);
    trySegment(best_idx, best_idx + 1);

    best_s = std::fmod(best_s, total_length_);
    if (best_s < 0.0) best_s += total_length_;

    return {best_s, best_signed_d};
}

std::pair<double, double> FrenetFrame::frenetToCartesian(double s, double d) const
{
    if (!initialized_) return {0.0, 0.0};

    s = std::fmod(s, total_length_);
    if (s < 0.0) s += total_length_;

    int idx = static_cast<int>(
        std::upper_bound(s_values_.begin(), s_values_.end(), s) - s_values_.begin()) - 1;
    idx = std::clamp(idx, 0, N_ - 1);
    int idx_next = (idx + 1) % N_;

    double seg_s = (idx == N_ - 1)
        ? (total_length_ - s_values_[N_-1])
        : (s_values_[idx_next] - s_values_[idx]);

    double t = (seg_s > 1e-9) ? ((s - s_values_[idx]) / seg_s) : 0.0;
    double px = centerline_[idx][0] + t * (centerline_[idx_next][0] - centerline_[idx][0]);
    double py = centerline_[idx][1] + t * (centerline_[idx_next][1] - centerline_[idx][1]);

    double heading = headings_[idx];
    return {px + d * (-std::sin(heading)), py + d * std::cos(heading)};
}

}  // namespace multiraceline
