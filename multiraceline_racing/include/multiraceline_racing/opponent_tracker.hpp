#pragma once
#include <optional>
#include <vector>
#include <cmath>
#include "multiraceline_racing/opponent_detector.hpp"
#include "multiraceline_racing/frenet_frame.hpp"

namespace multiraceline {

struct TrackedOpponent {
    double x, y;        // world frame position
    double s, d;        // frenet frame position (KF estimate)
    double vx, vy;      // world-frame velocity (m/s)
    double vs, vd;      // frenet velocity (along-track, lateral)
    double as_, ad;     // frenet acceleration (along-track, lateral)
    double s_sigma;     // 1-sigma position uncertainty along track (m) — sqrt(P[0][0])
    double d_sigma;     // 1-sigma position uncertainty lateral (m)
    double age;         // seconds since last matched detection
};

// 1-D constant-acceleration Kalman filter. State: [pos, vel, acc].
// Covariance is 3x3 stored row-major in p[3][3].
struct KF1D {
    double pos = 0.0, vel = 0.0, acc = 0.0;
    double p[3][3] = {{1.0, 0.0, 0.0},
                      {0.0, 1.0, 0.0},
                      {0.0, 0.0, 0.1}};

    // Tunable noise params (set by OpponentTracker from its Params)
    double q_pos = 0.05;   // process noise: position variance/s
    double q_vel = 0.5;    // process noise: velocity variance/s
    double q_acc = 2.0;    // process noise: acceleration variance/s
    double r_pos = 0.1;    // measurement noise: position variance

    // State transition + covariance predict step.
    void predict(double dt);

    // Measurement update with scalar position observation z.
    void update(double z);

    // Initialize from a known position (vel and acc start at 0, high covariance).
    void init(double position, double init_cov = 2.0);

    // Normalized innovation (Mahalanobis distance) for measurement z.
    // Returns sqrt((z - pos)^2 / (P[0][0] + R)).
    double mahalanobis(double z) const;

    // Position 1-sigma uncertainty.
    double sigma() const { return std::sqrt(std::max(0.0, p[0][0])); }
};

class OpponentTracker {
public:
    struct Params {
        // KF noise — increase q_acc to track harder braking/acceleration
        double kf_q_pos = 0.05;   // process noise: position (m²/s)
        double kf_q_vel = 0.5;    // process noise: velocity (m²/s³)
        double kf_q_acc = 2.0;    // process noise: acceleration (m²/s⁵)
        double kf_r_pos = 0.1;    // measurement noise: position (m²)

        // Track lifetime
        double max_age = 0.8;              // seconds before dropping a lost track
        double max_predicted_vs = 5.0;     // clamp predicted along-track speed (m/s)

        // Frenet gating — pre-filter before any KF math
        double gate_s = 3.0;   // max allowed |Δs| for a valid association (m)
        double gate_d = 1.0;   // max allowed |Δd| for a valid association (m)

        // Mahalanobis chi gate — normalized innovation threshold (1-D, ~99.7% = 3.0)
        double chi_threshold = 3.5;

        // Track wrap-around
        double track_length = 0.0;  // 0 = use 100m heuristic
    };

    explicit OpponentTracker(const Params& params);

    // Update tracker. When detections is empty, runs prediction-only and ages
    // the track. Returns nullopt if the track has been lost.
    std::optional<TrackedOpponent> update(
        const std::vector<DetectedObstacle>& detections,
        const FrenetFrame& frenet,
        double dt);

    // Nudge the KF's predicted along-track speed toward the raceline speed profile
    // at the opponent's last known position (call after an out-of-sight predict step).
    void setExpectedVs(double vs_expected);

    const std::optional<TrackedOpponent>& opponent() const { return opponent_; }

private:
    Params params_;
    std::optional<TrackedOpponent> opponent_;
    KF1D kf_s_, kf_d_;

    void initTrack(const DetectedObstacle& det, const FrenetFrame& frenet);
    void associateAndUpdate(const std::vector<DetectedObstacle>& dets,
                            const FrenetFrame& frenet, double dt);
    void writeBackState();
};

}  // namespace multiraceline
