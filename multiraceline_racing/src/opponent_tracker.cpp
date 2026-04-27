#include "multiraceline_racing/opponent_tracker.hpp"
#include <cmath>
#include <limits>

namespace multiraceline {

// ── KF1D — constant-acceleration model ───────────────────────────────────────
// State: x = [pos, vel, acc]
// Transition: F = [[1, dt, dt²/2], [0, 1, dt], [0, 0, 1]]
// Measurement: H = [1, 0, 0]

void KF1D::init(double position, double init_cov) {
    pos = position; vel = 0.0; acc = 0.0;
    p[0][0] = init_cov; p[0][1] = 0.0; p[0][2] = 0.0;
    p[1][0] = 0.0; p[1][1] = init_cov; p[1][2] = 0.0;
    p[2][0] = 0.0; p[2][1] = 0.0; p[2][2] = init_cov * 0.1;
}

void KF1D::predict(double dt) {
    const double h = 0.5 * dt * dt;

    // State propagation: x_new = F * x
    pos += vel * dt + acc * h;
    vel += acc * dt;
    // acc unchanged

    // Covariance: P_new = F*P*F^T + Q
    // Step 1 — FP = F * P (each row transformed by F)
    double fp[3][3];
    for (int j = 0; j < 3; j++) {
        fp[0][j] = p[0][j] + dt * p[1][j] + h * p[2][j];
        fp[1][j] = p[1][j] + dt * p[2][j];
        fp[2][j] = p[2][j];
    }
    // Step 2 — P = FP * F^T  (each column transformed by F^T = F's rows)
    for (int i = 0; i < 3; i++) {
        p[i][0] = fp[i][0] + dt * fp[i][1] + h * fp[i][2];
        p[i][1] = fp[i][1] + dt * fp[i][2];
        p[i][2] = fp[i][2];
    }
    // Step 3 — add diagonal process noise (scaled by dt so total noise is dt-independent)
    p[0][0] += q_pos * dt;
    p[1][1] += q_vel * dt;
    p[2][2] += q_acc * dt;
}

void KF1D::update(double z) {
    // S = H*P*H^T + R = P[0][0] + R
    double S = p[0][0] + r_pos;
    // Kalman gain: K[i] = P[i][0] / S  (H = [1,0,0])
    double k0 = p[0][0] / S;
    double k1 = p[1][0] / S;
    double k2 = p[2][0] / S;
    double innov = z - pos;
    pos += k0 * innov;
    vel += k1 * innov;
    acc += k2 * innov;
    // P_new = (I - K*H) * P  →  P[i][j] -= K[i] * P[0][j]
    for (int j = 0; j < 3; j++) {
        double p0j = p[0][j];
        p[0][j] -= k0 * p0j;
        p[1][j] -= k1 * p0j;
        p[2][j] -= k2 * p0j;
    }
}

double KF1D::mahalanobis(double z) const {
    double S = p[0][0] + r_pos;
    double innov = z - pos;
    return std::sqrt(innov * innov / S);
}

// ── OpponentTracker ───────────────────────────────────────────────────────────

OpponentTracker::OpponentTracker(const Params& params) : params_(params) {
    kf_s_.q_pos = kf_d_.q_pos = params_.kf_q_pos;
    kf_s_.q_vel = kf_d_.q_vel = params_.kf_q_vel;
    kf_s_.q_acc = kf_d_.q_acc = params_.kf_q_acc;
    kf_s_.r_pos = kf_d_.r_pos = params_.kf_r_pos;
}

void OpponentTracker::initTrack(const DetectedObstacle& det, const FrenetFrame& frenet) {
    auto fp = frenet.cartesianToFrenet(det.x, det.y);
    kf_s_.init(fp.s, 2.0);
    kf_d_.init(fp.d, 0.5);
    opponent_ = TrackedOpponent{};
    opponent_->x = det.x;
    opponent_->y = det.y;
    opponent_->s = fp.s;
    opponent_->d = fp.d;
    opponent_->vx = opponent_->vy = 0.0;
    opponent_->vs = opponent_->vd = 0.0;
    opponent_->as_ = opponent_->ad = 0.0;
    opponent_->s_sigma = kf_s_.sigma();
    opponent_->d_sigma = kf_d_.sigma();
    opponent_->age = 0.0;
}

void OpponentTracker::associateAndUpdate(
    const std::vector<DetectedObstacle>& dets,
    const FrenetFrame& frenet,
    double dt)
{
    double half = params_.track_length > 0.0 ? params_.track_length / 2.0 : 100.0;
    double wrap = params_.track_length > 0.0 ? params_.track_length : 200.0;

    double best_cost = std::numeric_limits<double>::max();
    const DetectedObstacle* best_det = nullptr;
    double best_s_meas = 0.0, best_d_meas = 0.0;

    for (const auto& det : dets) {
        // ── Improvement 3: Frenet gating ─────────────────────────────────────
        // Convert detection to Frenet and reject anything clearly outside the
        // predicted track extent. This is faster than KF math and eliminates
        // wall reflections and other cars that are far in s or d.
        auto fp = frenet.cartesianToFrenet(det.x, det.y);

        double ds = fp.s - kf_s_.pos;
        if (ds >  half) ds -= wrap;
        if (ds < -half) ds += wrap;
        double dd = fp.d - kf_d_.pos;

        if (std::abs(ds) > params_.gate_s || std::abs(dd) > params_.gate_d) continue;

        // ── Improvement 2: Mahalanobis gating ────────────────────────────────
        // Use innovation covariance (P[0][0] + R) to normalize the residual.
        // This accounts for KF uncertainty: a large P allows wider matches.
        double s_meas = kf_s_.pos + ds;  // wrap-corrected measurement
        double mah_s = kf_s_.mahalanobis(s_meas);
        double mah_d = kf_d_.mahalanobis(fp.d);

        if (mah_s > params_.chi_threshold || mah_d > params_.chi_threshold) continue;

        // Combined cost = sum of squared normalized innovations
        double cost = mah_s * mah_s + mah_d * mah_d;
        if (cost < best_cost) {
            best_cost   = cost;
            best_det    = &det;
            best_s_meas = s_meas;
            best_d_meas = fp.d;
        }
    }

    if (!best_det) return;  // no valid association

    // KF measurement update
    kf_s_.update(best_s_meas);
    kf_d_.update(best_d_meas);

    // World-frame velocity from position delta (save prev before overwriting)
    double prev_x = opponent_->x;
    double prev_y = opponent_->y;
    opponent_->x   = best_det->x;
    opponent_->y   = best_det->y;
    opponent_->age = 0.0;

    if (dt > 1e-6) {
        double raw_vx = (opponent_->x - prev_x) / dt;
        double raw_vy = (opponent_->y - prev_y) / dt;
        opponent_->vx = 0.8 * opponent_->vx + 0.2 * raw_vx;
        opponent_->vy = 0.8 * opponent_->vy + 0.2 * raw_vy;
    }
}

void OpponentTracker::writeBackState() {
    if (!opponent_) return;
    opponent_->s       = kf_s_.pos;
    opponent_->d       = kf_d_.pos;
    opponent_->vs      = kf_s_.vel;
    opponent_->vd      = kf_d_.vel;
    opponent_->as_     = kf_s_.acc;
    opponent_->ad      = kf_d_.acc;
    opponent_->s_sigma = kf_s_.sigma();
    opponent_->d_sigma = kf_d_.sigma();
}

std::optional<TrackedOpponent> OpponentTracker::update(
    const std::vector<DetectedObstacle>& detections,
    const FrenetFrame& frenet,
    double dt)
{
    if (!opponent_) {
        if (detections.empty()) return std::nullopt;
        // Initialize from the highest-confidence detection
        const DetectedObstacle* best = nullptr;
        for (const auto& d : detections) {
            if (!best || d.confidence > best->confidence) best = &d;
        }
        initTrack(*best, frenet);
        return opponent_;
    }

    // Prediction step (always runs — constant acceleration propagation)
    kf_s_.predict(dt);
    kf_d_.predict(dt);

    // Clamp predicted along-track speed to prevent unbounded drift
    kf_s_.vel = std::clamp(kf_s_.vel, -params_.max_predicted_vs, params_.max_predicted_vs);

    if (detections.empty()) {
        opponent_->age += dt;
        if (opponent_->age > params_.max_age) {
            opponent_.reset();
            return std::nullopt;
        }
    } else {
        associateAndUpdate(detections, frenet, dt);
    }

    if (!opponent_) return std::nullopt;
    writeBackState();
    return opponent_;
}

void OpponentTracker::setExpectedVs(double vs_expected) {
    if (!opponent_) return;
    // Gently nudge predicted along-track speed toward the known raceline speed
    // when the opponent is out of sight. Prevents unrealistic drift of the KF.
    kf_s_.vel = 0.9 * kf_s_.vel + 0.1 * vs_expected;
    opponent_->vs = kf_s_.vel;
}

}  // namespace multiraceline
