#include "multiraceline_racing/tactical_state_machine.hpp"
#include "multiraceline_racing/raceline_library.hpp"
#include <cmath>

namespace multiraceline {

std::string stateToString(TacticalState s)
{
    switch (s) {
        case TacticalState::SOLO_RACING:      return "SOLO_RACING";
        case TacticalState::FOLLOWING:        return "FOLLOWING";
        case TacticalState::OVERTAKING_LEFT:  return "OVERTAKING_LEFT";
        case TacticalState::OVERTAKING_RIGHT: return "OVERTAKING_RIGHT";
        case TacticalState::DEFENDING:        return "DEFENDING";
    }
    return "UNKNOWN";
}

TacticalStateMachine::TacticalStateMachine(const Params& params) : params_(params) {}

TacticalState TacticalStateMachine::computeDesired(
    double ego_s, double ego_d, double ego_speed,
    const std::optional<TrackedOpponent>& opp) const
{
    if (!opp) return TacticalState::SOLO_RACING;

    // Arc-length difference: positive = opponent is ahead
    double ds = opp->s - ego_s;

    // Track wrap-around
    double half = params_.track_length > 0.0 ? params_.track_length / 2.0 : 100.0;
    double wrap = params_.track_length > 0.0 ? params_.track_length : 200.0;
    if (ds >  half) ds -= wrap;
    if (ds < -half) ds += wrap;

    // ── CRITICAL: check ongoing overtake BEFORE the follow/close checks ───────
    // Without this, a widening gap (3–6 m ahead) would return FOLLOWING and
    // abort the overtake mid-manoeuvre.
    bool overtaking = (state_ == TacticalState::OVERTAKING_LEFT ||
                       state_ == TacticalState::OVERTAKING_RIGHT);
    if (overtaking) {
        if (ds < -params_.overtake_complete_s) {
            return TacticalState::SOLO_RACING;  // we've passed the opponent
        }
        if (ds > params_.follow_threshold_s) {
            return TacticalState::SOLO_RACING;  // opponent drove away, abort
        }
        return state_;  // keep current overtaking direction
    }

    // ── Opponent ahead ────────────────────────────────────────────────────────
    if (ds > 0.0 && ds < params_.follow_threshold_s) {

        // Velocity-predictive trigger: compute time-to-contact if closing.
        // opp->vs is the KF-estimated along-track speed of the opponent.
        // If we're closing fast, treat the gap as shorter so we engage sooner.
        double effective_ds = ds;
        if (params_.time_to_contact_threshold > 0.0 && ego_speed > 0.1) {
            double closing_rate = ego_speed - opp->vs;  // positive = closing
            if (closing_rate > 0.2) {
                double ttc = ds / closing_rate;
                if (ttc < params_.time_to_contact_threshold) {
                    // Closing fast — treat as already at overtake distance
                    effective_ds = params_.overtake_threshold_s * 0.5;
                }
            }
        }

        if (effective_ds < params_.overtake_threshold_s) {
            // Close enough — attempt pass if lateral gap exists on either side
            if (ego_d < opp->d - params_.min_lateral_gap) {
                return TacticalState::OVERTAKING_LEFT;
            }
            if (ego_d > opp->d + params_.min_lateral_gap) {
                return TacticalState::OVERTAKING_RIGHT;
            }
            // No lateral gap — stay in following and wait for an opening
        }
        return TacticalState::FOLLOWING;
    }

    // ── Opponent behind ───────────────────────────────────────────────────────
    if (ds < 0.0 && std::abs(ds) < params_.defend_threshold_s) {
        return TacticalState::DEFENDING;
    }

    return TacticalState::SOLO_RACING;
}

double TacticalStateMachine::holdTimeFor(TacticalState from, TacticalState to) const
{
    bool to_overtake = (to == TacticalState::OVERTAKING_LEFT ||
                        to == TacticalState::OVERTAKING_RIGHT);
    bool from_overtake = (from == TacticalState::OVERTAKING_LEFT ||
                          from == TacticalState::OVERTAKING_RIGHT);
    if (to_overtake)   return params_.overtake_entry_hold;
    if (from_overtake) return params_.overtake_exit_hold;
    return params_.state_hold_time;
}

std::string TacticalStateMachine::racelineFor(TacticalState s) const
{
    switch (s) {
        case TacticalState::SOLO_RACING:      return RacelineLibrary::OPTIMAL;
        case TacticalState::FOLLOWING:        return RacelineLibrary::DEFENSIVE;
        case TacticalState::OVERTAKING_LEFT:  return RacelineLibrary::INSIDE_ATTACK;
        case TacticalState::OVERTAKING_RIGHT: return RacelineLibrary::OUTSIDE_ATTACK;
        case TacticalState::DEFENDING:        return RacelineLibrary::DEFENSIVE;
    }
    return RacelineLibrary::OPTIMAL;
}

std::string TacticalStateMachine::update(
    double ego_s, double ego_d, double ego_speed,
    const std::optional<TrackedOpponent>& opponent,
    double dt,
    bool opponent_behind_override)
{
    // Hard override: if the operator signals the opponent is known to be behind,
    // snap immediately to SOLO_RACING without waiting for hysteresis.
    if (opponent_behind_override) {
        state_ = TacticalState::SOLO_RACING;
        pending_ = state_;
        hold_timer_ = 0.0;
        return racelineFor(state_);
    }

    TacticalState desired = computeDesired(ego_s, ego_d, ego_speed, opponent);

    if (desired != state_) {
        if (desired != pending_) {
            // New transition target — reset timer to avoid carrying over a
            // partial count from a previous oscillation.
            pending_ = desired;
            hold_timer_ = 0.0;
        }
        hold_timer_ += dt;
        if (hold_timer_ >= holdTimeFor(state_, pending_)) {
            state_ = pending_;
            hold_timer_ = 0.0;
        }
    } else {
        // Desired matches current — cancel any pending transition
        pending_ = state_;
        hold_timer_ = 0.0;
    }

    return racelineFor(state_);
}

}  // namespace multiraceline
