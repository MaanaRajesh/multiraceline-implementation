#include "multiraceline_racing/opponent_tracker.hpp"
#include <cmath>
#include <limits>

namespace multiraceline {

// ── KF1D ─────────────────────────────────────────────────────────────────────

void KF1D::init(double p, double v, double cov) {
    pos = p; vel = v;
    p00 = cov; p01 = 0.0; p10 = 0.0; p11 = cov;
}

void KF1D::predict(double dt) {
    // State: x_new = F*x, F = [[1,dt],[0,1]]
    pos += vel * dt;
    // Covariance: P_new = F*P*F^T + Q (Q diagonal: [q_pos*dt^2, q_vel*dt])
    double new_p00 = p00 + dt * (p01 + p10) + dt * dt * p11 + q_pos * dt * dt;
    double new_p01 = p01 + dt * p11;
    double new_p10 = p10 + dt * p11;
    double new_p11 = p11 + q_vel * dt;
    p00 = new_p00; p01 = new_p01; p10 = new_p10; p11 = new_p11;
}

void KF1D::update(double z) {
    // H = [1, 0], so S = p00 + R
    double S = p00 + r_pos;
    double k0 = p00 / S;  // Kalman gain for position
    double k1 = p10 / S;  // Kalman gain for velocity
    double innov = z - pos;
    pos += k0 * innov;
    vel += k1 * innov;
    // Joseph form for numerical stability
    double new_p00 = (1.0 - k0) * p00;
    double new_p01 = (1.0 - k0) * p01;
    double new_p10 = p10 - k1 * p00;
    double new_p11 = p11 - k1 * p01;
    p00 = new_p00; p01 = new_p01; p10 = new_p10; p11 = new_p11;
}

// ── OpponentTracker ───────────────────────────────────────────────────────────

OpponentTracker::OpponentTracker(const Params& params) : params_(params) {
    kf_s_.q_pos = kf_d_.q_pos = params_.kf_q_pos;
    kf_s_.q_vel = kf_d_.q_vel = params_.kf_q_vel;
    kf_s_.r_pos = kf_d_.r_pos = params_.kf_r_pos;
}

void OpponentTracker::initTrack(const DetectedObstacle& det, const FrenetFrame& frenet) {
    auto fp = frenet.cartesianToFrenet(det.x, det.y);
    kf_s_.init(fp.s, 0.0, 2.0);
    kf_d_.init(fp.d, 0.0, 0.5);
    opponent_ = TrackedOpponent{det.x, det.y, fp.s, fp.d, 0.0, 0.0, 0.0, 0.0, 0.0};
}

void OpponentTracker::associateAndUpdate(
    const std::vector<DetectedObstacle>& dets,
    const FrenetFrame& frenet,
    double dt)
{
    // Data association: find closest detection to predicted position
    double best_dist = std::numeric_limits<double>::max();
    const DetectedObstacle* best = nullptr;

    for (const auto& d : dets) {
        double dist = std::hypot(d.x - opponent_->x, d.y - opponent_->y);
        if (dist < best_dist && dist < params_.max_association_dist) {
            best_dist = dist;
            best = &d;
        }
    }

    if (!best) {
        // No association — prediction-only step (already done before this call)
        return;
    }

    // Measurement update
    auto fp = frenet.cartesianToFrenet(best->x, best->y);

    // Handle s wrap-around: pick the measurement closest to predicted s
    double half = params_.track_length > 0.0 ? params_.track_length / 2.0 : 100.0;
    double wrap = params_.track_length > 0.0 ? params_.track_length : 200.0;
    double ds_meas = fp.s - kf_s_.pos;
    if (ds_meas >  half) ds_meas -= wrap;
    if (ds_meas < -half) ds_meas += wrap;
    double s_meas_wrapped = kf_s_.pos + ds_meas;

    kf_s_.update(s_meas_wrapped);
    kf_d_.update(fp.d);

    // World-frame velocity from position delta (save prev before overwriting)
    double prev_x = opponent_->x;
    double prev_y = opponent_->y;
    opponent_->x = best->x;
    opponent_->y = best->y;
    opponent_->age = 0.0;

    if (dt > 1e-6) {
        double raw_vx = (opponent_->x - prev_x) / dt;
        double raw_vy = (opponent_->y - prev_y) / dt;
        opponent_->vx = 0.8 * opponent_->vx + 0.2 * raw_vx;
        opponent_->vy = 0.8 * opponent_->vy + 0.2 * raw_vy;
    }
}

std::optional<TrackedOpponent> OpponentTracker::update(
    const std::vector<DetectedObstacle>& detections,
    const FrenetFrame& frenet,
    double dt)
{
    if (!opponent_) {
        // No track: initialize from highest-confidence detection
        if (detections.empty()) return std::nullopt;
        const DetectedObstacle* best = nullptr;
        for (const auto& d : detections) {
            if (!best || d.confidence > best->confidence) best = &d;
        }
        initTrack(*best, frenet);
        return opponent_;
    }

    // Prediction step (always runs)
    kf_s_.predict(dt);
    kf_d_.predict(dt);

    // Clamp predicted vs to max_predicted_vs to prevent drift
    kf_s_.vel = std::clamp(kf_s_.vel, -params_.max_predicted_vs, params_.max_predicted_vs);

    if (detections.empty()) {
        // Age the track
        opponent_->age += dt;
        if (opponent_->age > params_.max_age) {
            opponent_.reset();
            return std::nullopt;
        }
    } else {
        // Measurement update
        associateAndUpdate(detections, frenet, dt);
    }

    if (!opponent_) return std::nullopt;

    // Write KF state back into tracked opponent
    opponent_->s  = kf_s_.pos;
    opponent_->d  = kf_d_.pos;
    opponent_->vs = kf_s_.vel;
    opponent_->vd = kf_d_.vel;

    return opponent_;
}

void OpponentTracker::setExpectedVs(double vs_expected) {
    if (!opponent_) return;
    // Gently nudge KF velocity toward the expected raceline speed when out of sight.
    // Constrains drift: opponent is assumed to follow the known speed profile.
    kf_s_.vel = 0.9 * kf_s_.vel + 0.1 * vs_expected;
    opponent_->vs = kf_s_.vel;
}

}  // namespace multiraceline
