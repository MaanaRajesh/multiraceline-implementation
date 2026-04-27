#pragma once
#include <optional>
#include <vector>
#include "multiraceline_racing/opponent_detector.hpp"
#include "multiraceline_racing/frenet_frame.hpp"

namespace multiraceline {

struct TrackedOpponent {
    double x, y;      // world frame (KF-filtered)
    double s, d;      // frenet frame (KF-filtered)
    double vx, vy;    // velocity in world frame (m/s)
    double vs, vd;    // velocity in frenet frame (along-track, lateral)
    double age;       // seconds since last matched detection
};

// Independent 1-D constant-velocity Kalman filter (position + velocity state).
struct KF1D {
    double pos = 0.0, vel = 0.0;
    // 2x2 covariance [p00 p01 / p10 p11]
    double p00 = 1.0, p01 = 0.0, p10 = 0.0, p11 = 1.0;

    // q_pos / q_vel: process noise variances per second for position / velocity.
    // r_pos: measurement noise variance.
    double q_pos = 0.05, q_vel = 0.5, r_pos = 0.1;

    void predict(double dt);
    void update(double z);
    void init(double p, double v, double cov);
};

class OpponentTracker {
public:
    struct Params {
        // Kalman filter noise parameters
        double kf_q_pos = 0.05;            // process noise: position (m²/s)
        double kf_q_vel = 0.5;             // process noise: velocity (m²/s³)
        double kf_r_pos = 0.1;             // measurement noise: position (m²)
        // Association & lifetime
        double max_age = 0.8;              // drop track after this many seconds without detection
        double max_association_dist = 1.0; // max match distance (m) for data association
        double track_length = 0.0;         // total track length for s wrap-around (0 = 100m heuristic)
        // When out of sight, clamp predicted along-track speed to this maximum (m/s).
        // Set to the opponent's expected top speed to prevent unbounded drift.
        double max_predicted_vs = 5.0;
    };

    explicit OpponentTracker(const Params& params);

    // Update tracker with new detections. When detections is empty, runs
    // KF prediction only and ages the track. Returns nullopt when track is lost.
    std::optional<TrackedOpponent> update(
        const std::vector<DetectedObstacle>& detections,
        const FrenetFrame& frenet,
        double dt);

    // Override the predicted along-track speed (call after update when out of sight
    // to constrain the KF velocity to the raceline speed profile at that s position).
    void setExpectedVs(double vs_expected);

    const std::optional<TrackedOpponent>& opponent() const { return opponent_; }

private:
    Params params_;
    std::optional<TrackedOpponent> opponent_;

    // KF instances for s and d (frenet). Only valid when opponent_ is set.
    KF1D kf_s_, kf_d_;

    void initTrack(const DetectedObstacle& det, const FrenetFrame& frenet);
    void associateAndUpdate(const std::vector<DetectedObstacle>& dets,
                            const FrenetFrame& frenet, double dt);
};

}  // namespace multiraceline
