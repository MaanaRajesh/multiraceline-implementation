#pragma once
#include <string>
#include <optional>
#include "multiraceline_racing/opponent_tracker.hpp"

namespace multiraceline {

enum class TacticalState {
    SOLO_RACING,
    FOLLOWING,
    OVERTAKING_LEFT,
    OVERTAKING_RIGHT,
    DEFENDING,
};

std::string stateToString(TacticalState s);

class TacticalStateMachine {
public:
    struct Params {
        // Distance thresholds (meters in Frenet s)
        double follow_threshold_s    = 6.0;  // engage following if opponent < N m ahead
        double overtake_threshold_s  = 3.0;  // attempt overtake if < N m ahead
        double overtake_complete_s   = 2.0;  // done when we're N m ahead of opponent
        double defend_threshold_s    = 4.0;  // defend if opponent < N m behind

        // Overtake geometry
        double min_lateral_gap = 0.5;        // required lateral clearance (m) for attempt

        // Hysteresis — asymmetric: commit to overtake quickly, release slowly
        double state_hold_time    = 0.5;     // default hold before any transition (s)
        double overtake_entry_hold = 0.2;    // shorter hold for entering overtake (s)
        double overtake_exit_hold  = 1.2;    // longer hold before abandoning overtake (s)

        // Velocity-predictive overtake trigger
        // If TTC (time-to-contact) < this threshold, treat as close enough to overtake.
        // Set to 0 to disable. Requires ego_speed to be passed to update().
        double time_to_contact_threshold = 2.5;  // seconds

        // Track geometry
        double track_length = 0.0;           // 0 = use 200m heuristic for wrap-around
    };

    explicit TacticalStateMachine(const Params& params);

    // Returns selected raceline name.
    // ego_speed: car's current forward speed (m/s). Used for TTC check; pass 0 if unknown.
    // opponent_behind_override: if true (from button), force SOLO_RACING immediately.
    std::string update(
        double ego_s,
        double ego_d,
        double ego_speed,
        const std::optional<TrackedOpponent>& opponent,
        double dt,
        bool opponent_behind_override = false);

    TacticalState currentState() const { return state_; }

private:
    Params params_;
    TacticalState state_   = TacticalState::SOLO_RACING;
    TacticalState pending_ = TacticalState::SOLO_RACING;
    double hold_timer_ = 0.0;

    TacticalState computeDesired(
        double ego_s, double ego_d, double ego_speed,
        const std::optional<TrackedOpponent>& opponent) const;

    std::string racelineFor(TacticalState s) const;

    // Returns the required hold time for a transition from→to.
    double holdTimeFor(TacticalState from, TacticalState to) const;
};

}  // namespace multiraceline
