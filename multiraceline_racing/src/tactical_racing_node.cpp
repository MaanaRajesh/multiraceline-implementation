#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "multiraceline_racing/frenet_frame.hpp"
#include "multiraceline_racing/raceline_library.hpp"
#include "multiraceline_racing/opponent_detector.hpp"
#include "multiraceline_racing/opponent_tracker.hpp"
#include "multiraceline_racing/tactical_state_machine.hpp"
#include "multiraceline_racing/pure_pursuit_controller.hpp"

using namespace multiraceline;

class TacticalRacingNode : public rclcpp::Node
{
public:
    TacticalRacingNode() : Node("tactical_racing_node")
    {
        // ── Parameters ──────────────────────────────────────────────────────
        declare_parameter("use_sim", true);

        // Raceline CSV paths
        declare_parameter("centerline_csv",    "");
        declare_parameter("optimal_csv",       "");
        declare_parameter("inside_attack_csv", "");
        declare_parameter("outside_attack_csv","");
        declare_parameter("defensive_csv",     "");

        // Pure Pursuit
        declare_parameter("lookahead_min",      0.5);
        declare_parameter("lookahead_max",      2.0);
        declare_parameter("fast_speed",         2.0);
        declare_parameter("slow_speed",         0.5);
        declare_parameter("steering_limit",     0.4189);
        declare_parameter("steer_alpha",        0.8);
        declare_parameter("use_waypoint_speed", true);

        // Opponent detection
        declare_parameter("detection_max_range",    5.0);
        declare_parameter("detection_cluster_gap",  0.3);
        declare_parameter("detection_min_points",   3);
        declare_parameter("detection_max_width",    0.6);

        // Opponent tracker (KF)
        declare_parameter("tracker_kf_q_pos",           0.05);
        declare_parameter("tracker_kf_q_vel",           0.5);
        declare_parameter("tracker_kf_r_pos",           0.1);
        declare_parameter("tracker_max_age",             0.8);
        declare_parameter("tracker_max_association_dist",1.0);
        declare_parameter("tracker_max_predicted_vs",    5.0);

        // State machine
        declare_parameter("follow_threshold_s",        6.0);
        declare_parameter("overtake_threshold_s",      3.0);
        declare_parameter("overtake_complete_s",       2.0);
        declare_parameter("defend_threshold_s",        4.0);
        declare_parameter("min_lateral_gap",           0.5);
        declare_parameter("state_hold_time",           0.5);
        declare_parameter("overtake_entry_hold",       0.2);
        declare_parameter("overtake_exit_hold",        1.2);
        declare_parameter("time_to_contact_threshold", 2.5);

        // ── Build components ─────────────────────────────────────────────────
        loadRacelines();
        buildDetector();
        buildStateMachine();
        buildPurePursuit();

        // ── Pubs / Subs ───────────────────────────────────────────────────────
        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>("/tactical_state", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/tactical_markers", 10);

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&TacticalRacingNode::scanCallback, this, std::placeholders::_1));

        // Manual override: operator signals opponent is confirmed behind
        opponent_behind_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/opponent_behind", 10,
            [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
                opponent_behind_ = msg->data;
                if (msg->data) {
                    RCLCPP_INFO(get_logger(), "Operator: opponent confirmed behind — forcing SOLO_RACING");
                }
            });

        bool use_sim = get_parameter("use_sim").as_bool();
        if (use_sim) {
            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "/ego_racecar/odom", 10,
                std::bind(&TacticalRacingNode::odomCallback, this, std::placeholders::_1));
        } else {
            pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                "/pf/viz/inferred_pose", 10,
                std::bind(&TacticalRacingNode::poseCallback, this, std::placeholders::_1));
        }

        RCLCPP_INFO(get_logger(), "TacticalRacingNode initialized. Active raceline: %s",
                    active_raceline_.c_str());
    }

private:
    // ── Components ────────────────────────────────────────────────────────────
    RacelineLibrary library_;
    std::unique_ptr<OpponentDetector> detector_;
    std::unique_ptr<OpponentTracker>  tracker_;
    std::unique_ptr<TacticalStateMachine> state_machine_;
    std::unique_ptr<PurePursuitController> pursuit_;

    // ── State ─────────────────────────────────────────────────────────────────
    double car_x_ = 0.0, car_y_ = 0.0, car_yaw_ = 0.0;
    double car_speed_ = 0.0;  // forward speed from odometry (m/s)
    bool   opponent_behind_ = false;
    std::string active_raceline_ = RacelineLibrary::OPTIMAL;
    rclcpp::Time prev_time_;
    bool prev_time_valid_ = false;

    std::vector<float> latest_ranges_;
    float scan_angle_min_ = 0.0f;
    float scan_angle_increment_ = 0.0f;

    // ── ROS interfaces ────────────────────────────────────────────────────────
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr opponent_behind_sub_;

    // ── Init helpers ──────────────────────────────────────────────────────────
    void loadRacelines()
    {
        auto cl  = get_parameter("centerline_csv").as_string();
        auto opt = get_parameter("optimal_csv").as_string();
        auto ins = get_parameter("inside_attack_csv").as_string();
        auto out = get_parameter("outside_attack_csv").as_string();
        auto def = get_parameter("defensive_csv").as_string();

        if (!cl.empty() && !library_.loadCenterline(cl)) {
            RCLCPP_WARN(get_logger(), "Failed to load centerline: %s", cl.c_str());
        }
        auto tryLoad = [&](const std::string& name, const std::string& path) {
            if (!path.empty() && !library_.loadRaceline(name, path)) {
                RCLCPP_WARN(get_logger(), "Failed to load raceline '%s': %s",
                            name.c_str(), path.c_str());
            }
        };
        tryLoad(RacelineLibrary::OPTIMAL,       opt);
        tryLoad(RacelineLibrary::INSIDE_ATTACK, ins);
        tryLoad(RacelineLibrary::OUTSIDE_ATTACK,out);
        tryLoad(RacelineLibrary::DEFENSIVE,     def);

        for (const auto& name : {RacelineLibrary::INSIDE_ATTACK,
                                  RacelineLibrary::OUTSIDE_ATTACK,
                                  RacelineLibrary::DEFENSIVE}) {
            if (!library_.hasRaceline(name) && library_.hasRaceline(RacelineLibrary::OPTIMAL)) {
                RCLCPP_WARN(get_logger(),
                    "Raceline '%s' not loaded — falling back to optimal", name.c_str());
            }
        }
    }

    void buildDetector()
    {
        OpponentDetector::Params p;
        p.max_range          = get_parameter("detection_max_range").as_double();
        p.cluster_gap        = get_parameter("detection_cluster_gap").as_double();
        p.min_cluster_points = get_parameter("detection_min_points").as_int();
        p.max_obstacle_width = get_parameter("detection_max_width").as_double();
        detector_ = std::make_unique<OpponentDetector>(p);

        OpponentTracker::Params tp;
        tp.kf_q_pos            = get_parameter("tracker_kf_q_pos").as_double();
        tp.kf_q_vel            = get_parameter("tracker_kf_q_vel").as_double();
        tp.kf_r_pos            = get_parameter("tracker_kf_r_pos").as_double();
        tp.max_age             = get_parameter("tracker_max_age").as_double();
        tp.max_association_dist= get_parameter("tracker_max_association_dist").as_double();
        tp.max_predicted_vs    = get_parameter("tracker_max_predicted_vs").as_double();
        tp.track_length        = library_.frenetFrame().totalLength();
        tracker_ = std::make_unique<OpponentTracker>(tp);
    }

    void buildStateMachine()
    {
        TacticalStateMachine::Params p;
        p.follow_threshold_s        = get_parameter("follow_threshold_s").as_double();
        p.overtake_threshold_s      = get_parameter("overtake_threshold_s").as_double();
        p.overtake_complete_s       = get_parameter("overtake_complete_s").as_double();
        p.defend_threshold_s        = get_parameter("defend_threshold_s").as_double();
        p.min_lateral_gap           = get_parameter("min_lateral_gap").as_double();
        p.state_hold_time           = get_parameter("state_hold_time").as_double();
        p.overtake_entry_hold       = get_parameter("overtake_entry_hold").as_double();
        p.overtake_exit_hold        = get_parameter("overtake_exit_hold").as_double();
        p.time_to_contact_threshold = get_parameter("time_to_contact_threshold").as_double();
        p.track_length              = library_.frenetFrame().totalLength();
        state_machine_ = std::make_unique<TacticalStateMachine>(p);
    }

    void buildPurePursuit()
    {
        PurePursuitController::Params p;
        p.lookahead_min      = get_parameter("lookahead_min").as_double();
        p.lookahead_max      = get_parameter("lookahead_max").as_double();
        p.fast_speed         = get_parameter("fast_speed").as_double();
        p.slow_speed         = get_parameter("slow_speed").as_double();
        p.steering_limit     = get_parameter("steering_limit").as_double();
        p.steer_alpha        = get_parameter("steer_alpha").as_double();
        p.use_waypoint_speed = get_parameter("use_waypoint_speed").as_bool();
        pursuit_ = std::make_unique<PurePursuitController>(p);
    }

    // ── Core loop ─────────────────────────────────────────────────────────────
    void runTacticalLoop(double x, double y, double yaw)
    {
        auto now = get_clock()->now();
        double dt = 0.05;
        if (prev_time_valid_) {
            dt = std::clamp((now - prev_time_).seconds(), 0.001, 0.5);
        }
        prev_time_ = now;
        prev_time_valid_ = true;

        car_x_ = x; car_y_ = y; car_yaw_ = yaw;

        // Opponent detection + KF tracking
        std::optional<TrackedOpponent> opponent;
        if (!latest_ranges_.empty() && library_.frenetFrame().isInitialized()) {
            auto dets = detector_->detect(
                latest_ranges_, scan_angle_min_, scan_angle_increment_,
                car_x_, car_y_, car_yaw_);

            opponent = tracker_->update(dets, library_.frenetFrame(), dt);

            // When opponent is out of sight, constrain KF velocity to the
            // expected speed on the optimal raceline at the opponent's last s.
            if (dets.empty() && opponent) {
                double expected_vs = lookupRacelineSpeedAt(opponent->s);
                tracker_->setExpectedVs(expected_vs);
            }
        }

        // Ego position in Frenet
        double ego_s = 0.0, ego_d = 0.0;
        if (library_.frenetFrame().isInitialized()) {
            auto fp = library_.frenetFrame().cartesianToFrenet(car_x_, car_y_);
            ego_s = fp.s;
            ego_d = fp.d;
        }

        // State machine → raceline selection
        std::string desired_raceline = state_machine_->update(
            ego_s, ego_d, car_speed_, opponent, dt, opponent_behind_);

        if (!library_.hasRaceline(desired_raceline)) {
            desired_raceline = RacelineLibrary::OPTIMAL;
        }

        if (desired_raceline != active_raceline_) {
            RCLCPP_INFO(get_logger(), "Raceline switch: %s → %s  (state: %s)",
                active_raceline_.c_str(), desired_raceline.c_str(),
                stateToString(state_machine_->currentState()).c_str());
            active_raceline_ = desired_raceline;
            pursuit_->resetIndex();
        }

        const auto& wps = library_.getRaceline(active_raceline_);
        if (!wps.empty()) {
            auto cmd = pursuit_->compute(car_x_, car_y_, car_yaw_, wps);
            publishDrive(cmd.steering_angle, cmd.speed);
        }

        publishState();
        publishMarkers(opponent);
    }

    // Look up the target speed on the optimal raceline at arc-length s.
    // Returns fast_speed if no raceline is available.
    double lookupRacelineSpeedAt(double s) const
    {
        const auto& wps = library_.getRaceline(RacelineLibrary::OPTIMAL);
        if (wps.empty()) return get_parameter("fast_speed").as_double();

        // Find the waypoint with the closest s value.
        // Waypoints are stored as [x, y, yaw, speed]; we need arc-length.
        // Since we don't store s in the waypoint directly, use nearest index
        // as a rough proxy (works when spacing is uniform).
        // A more accurate version would require the raceline s values to be stored.
        // For now, return the speed at the nearest-by-index waypoint.
        double total_len = library_.frenetFrame().totalLength();
        if (total_len <= 0.0) return get_parameter("fast_speed").as_double();
        double frac = std::fmod(s, total_len) / total_len;
        std::size_t idx = static_cast<std::size_t>(frac * wps.size());
        idx = std::min(idx, wps.size() - 1);
        // Waypoint column 3 is speed ratio (0–1); scale to fast_speed
        double fast_speed = get_parameter("fast_speed").as_double();
        return wps[idx][3] * fast_speed;
    }

    // ── Publishers ────────────────────────────────────────────────────────────
    void publishDrive(double steering, double speed)
    {
        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = "base_link";
        msg.drive.steering_angle = steering;
        msg.drive.speed = speed;
        drive_pub_->publish(msg);
    }

    void publishState()
    {
        std_msgs::msg::String msg;
        msg.data = stateToString(state_machine_->currentState()) + " [" + active_raceline_ + "]";
        state_pub_->publish(msg);
    }

    void publishMarkers(const std::optional<TrackedOpponent>& opponent)
    {
        visualization_msgs::msg::MarkerArray arr;

        if (opponent) {
            visualization_msgs::msg::Marker m;
            m.header.stamp = get_clock()->now();
            m.header.frame_id = "map";
            m.ns = "opponent";
            m.id = 0;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = opponent->x;
            m.pose.position.y = opponent->y;
            m.pose.position.z = 0.1;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = 0.3;
            m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 1.0;
            arr.markers.push_back(m);

            // Velocity arrow
            visualization_msgs::msg::Marker vel;
            vel.header = m.header;
            vel.ns = "opponent_vel";
            vel.id = 2;
            vel.type = visualization_msgs::msg::Marker::ARROW;
            vel.action = visualization_msgs::msg::Marker::ADD;
            vel.pose.position = m.pose.position;
            vel.pose.orientation.w = 1.0;
            vel.scale.x = std::hypot(opponent->vx, opponent->vy) * 0.5;
            vel.scale.y = 0.05; vel.scale.z = 0.05;
            vel.color.r = 1.0; vel.color.g = 0.6; vel.color.b = 0.0; vel.color.a = 0.8;
            arr.markers.push_back(vel);
        }

        const auto& wps = library_.getRaceline(active_raceline_);
        if (!wps.empty()) {
            visualization_msgs::msg::Marker line;
            line.header.stamp = get_clock()->now();
            line.header.frame_id = "map";
            line.ns = "active_raceline";
            line.id = 1;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.scale.x = 0.05;
            line.pose.orientation.w = 1.0;

            auto s = state_machine_->currentState();
            if (s == TacticalState::SOLO_RACING)
                { line.color.r=0.0; line.color.g=1.0; line.color.b=0.0; }
            else if (s == TacticalState::FOLLOWING)
                { line.color.r=1.0; line.color.g=1.0; line.color.b=0.0; }
            else if (s == TacticalState::OVERTAKING_LEFT ||
                     s == TacticalState::OVERTAKING_RIGHT)
                { line.color.r=0.0; line.color.g=0.5; line.color.b=1.0; }
            else
                { line.color.r=1.0; line.color.g=0.5; line.color.b=0.0; }
            line.color.a = 0.8;

            for (const auto& wp : wps) {
                geometry_msgs::msg::Point p;
                p.x = wp[0]; p.y = wp[1]; p.z = 0.02;
                line.points.push_back(p);
            }
            if (!wps.empty()) {
                geometry_msgs::msg::Point p;
                p.x = wps[0][0]; p.y = wps[0][1]; p.z = 0.02;
                line.points.push_back(p);
            }
            arr.markers.push_back(line);
        }

        marker_pub_->publish(arr);
    }

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
    {
        latest_ranges_        = msg->ranges;
        scan_angle_min_       = msg->angle_min;
        scan_angle_increment_ = msg->angle_increment;
    }

    static double quaternionToYaw(double qx, double qy, double qz, double qw)
    {
        return std::atan2(2.0 * (qw * qz + qx * qy),
                          1.0 - 2.0 * (qy * qy + qz * qz));
    }

    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        double yaw = quaternionToYaw(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        // Forward speed from body-frame twist
        car_speed_ = msg->twist.twist.linear.x;
        runTacticalLoop(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            yaw);
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        double yaw = quaternionToYaw(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        // No speed available from PoseStamped — TTC check is disabled (car_speed_=0)
        runTacticalLoop(msg->pose.position.x, msg->pose.position.y, yaw);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TacticalRacingNode>());
    rclcpp::shutdown();
    return 0;
}
