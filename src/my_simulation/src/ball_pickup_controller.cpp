#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <xmlrpcpp/XmlRpcValue.h>

struct BallInfo {
    std::string name;
    double world_x = 0.0;
    double world_y = 0.0;
    bool world_estimate_initialized = false;
    double model_world_x = 0.0;
    double model_world_y = 0.0;
    bool model_visible = false;
    bool visible = false;
    bool picked = false;
    ros::Time last_seen;
    bool initial_recorded = false;
    double initial_x = 0.0;
    double initial_y = 0.0;
    double pickup_time_sec = -1.0;
    int pickup_order = 0;
};

struct PathSample {
    double sim_time_sec = 0.0;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

class BallPickupController {
public:
    BallPickupController()
        : nh_(),
          pnh_("~"),
          delete_model_client_(nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model")) {
        loadParameters();

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        pickup_complete_pub_ = pnh_.advertise<std_msgs::Bool>("pickup_complete", 1, true);
        model_states_sub_ = nh_.subscribe("/gazebo/model_states", 1, &BallPickupController::modelStatesCallback, this);
        stereo_ball_positions_sub_ = nh_.subscribe("/stereo/ball_positions", 1, &BallPickupController::stereoBallPositionsCallback, this);
        start_service_ = pnh_.advertiseService("start", &BallPickupController::handleStartRequest, this);
        control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_),
                                         &BallPickupController::controlTimerCallback,
                                         this);

        for (const std::string& ball_name : ball_names_) {
            BallInfo info;
            info.name = ball_name;
            balls_[ball_name] = info;
        }
        loadSceneBallAnchors();

        publishPickupComplete(false);

        ROS_INFO("ball_pickup_controller started. strategy=%s, planning_source=%s, robot_model=%s, balls=%zu, wait_for_manual_start=%s",
                 strategy_.c_str(), planning_source_.c_str(), robot_model_name_.c_str(), ball_names_.size(),
                 wait_for_manual_start_ ? "true" : "false");
    }

    ~BallPickupController() {
        stopRobot();
        exportArtifacts();
    }

private:
    enum class ControlMode {
        ROTATE_ONLY,
        ADVANCE
    };

    enum class BehaviorState {
        WAITING_START,
        WAITING_DELAY,
        SEARCHING,
        TRAVERSING,
        TRACKING,
        PICKUP_SUCCESS,
        STOPPED_NO_TARGET
    };

    bool isPointStrategy() const {
        return strategy_ == "point";
    }

    bool useModelPlanning() const {
        return planning_source_ == "model";
    }

    std::string strategyDisplayName() const {
        return isPointStrategy() ? "traditional_point" : "improved_segment";
    }

    void loadParameters() {
        pnh_.param<std::string>("robot_model_name", robot_model_name_, "simple_car");
        pnh_.param<std::string>("strategy", strategy_, "segment");
        pnh_.param<std::string>("planning_source", planning_source_, "model");
        pnh_.param("control_frequency", control_frequency_, 20.0);
        pnh_.param("max_linear_speed", max_linear_speed_, 0.4);
        pnh_.param("max_angular_speed", max_angular_speed_, 0.8);
        pnh_.param("heading_gain", heading_gain_, 1.2);
        pnh_.param("distance_gain", distance_gain_, 0.5);
        pnh_.param("front_boundary_x", front_boundary_x_, 0.6917);
        pnh_.param("pickup_half_width", pickup_half_width_, 0.25);
        pnh_.param("ball_radius", ball_radius_, 0.033);
        pnh_.param("pickup_tolerance", pickup_tolerance_, 0.02);
        pnh_.param("x_error_margin", x_error_margin_, 0.0241);
        pnh_.param("depth_error_margin", depth_error_margin_, 0.0356);
        pnh_.param("segment_pickup_side_shrink", segment_pickup_side_shrink_, 0.0356);
        pnh_.param("segment_guidance_inset", segment_guidance_inset_, 0.0356);
        pnh_.param("shared_advance_lateral_overflow_tolerance", shared_advance_lateral_overflow_tolerance_, 0.04);
        pnh_.param("shared_advance_heading_tolerance", shared_advance_heading_tolerance_, 0.14);
        pnh_.param("shared_realign_lateral_overflow_tolerance", shared_realign_lateral_overflow_tolerance_, 0.10);
        pnh_.param("shared_realign_heading_tolerance", shared_realign_heading_tolerance_, 0.28);
        pnh_.param("shared_realign_forward_error_threshold", shared_realign_forward_error_threshold_, 0.20);
        pnh_.param("shared_advance_heading_deadband", shared_advance_heading_deadband_, 0.06);
        pnh_.param("shared_advance_lateral_deadband", shared_advance_lateral_deadband_, 0.02);
        pnh_.param("shared_advance_min_linear_speed", shared_advance_min_linear_speed_, 0.08);
        pnh_.param("shared_advance_min_angular_speed", shared_advance_min_angular_speed_, 0.05);
        pnh_.param("shared_advance_max_angular_speed", shared_advance_max_angular_speed_, 0.18);
        pnh_.param("shared_rotate_min_angular_speed", shared_rotate_min_angular_speed_, 0.10);
        pnh_.param("enable_recovery", enable_recovery_, false);
        pnh_.param("point_pick_heading_tolerance", point_pick_heading_tolerance_, 0.10);
        pnh_.param("point_pick_lateral_tolerance", point_pick_lateral_tolerance_, 0.05);
        pnh_.param("point_stuck_timeout_sec", point_stuck_timeout_sec_, 1.5);
        pnh_.param("point_progress_score_epsilon", point_progress_score_epsilon_, 0.015);
        pnh_.param("point_recovery_forward_speed", point_recovery_forward_speed_, 0.08);
        pnh_.param("point_recovery_turn_speed", point_recovery_turn_speed_, 0.45);
        pnh_.param("point_recovery_duration_sec", point_recovery_duration_sec_, 0.60);
        pnh_.param("point_heading_lookahead", point_heading_lookahead_, 0.35);
        pnh_.param("point_close_advance_window", point_close_advance_window_, 0.10);
        pnh_.param("point_close_lateral_exit_tolerance", point_close_lateral_exit_tolerance_, 0.12);
        pnh_.param("point_advance_max_angular_speed", point_advance_max_angular_speed_, 0.16);
        pnh_.param("advance_heading_deadband", advance_heading_deadband_, 0.08);
        pnh_.param("point_advance_lateral_deadband", point_advance_lateral_deadband_, 0.05);
        pnh_.param("point_advance_enter_heading_tolerance", point_advance_enter_heading_tolerance_, 0.22);
        pnh_.param("point_advance_enter_lateral_tolerance", point_advance_enter_lateral_tolerance_, 0.10);
        pnh_.param("point_advance_exit_heading_tolerance", point_advance_exit_heading_tolerance_, 0.45);
        pnh_.param("point_advance_exit_lateral_tolerance", point_advance_exit_lateral_tolerance_, 0.18);
        pnh_.param("point_rotate_recover_forward_threshold", point_rotate_recover_forward_threshold_, 0.45);
        pnh_.param("segment_advance_lateral_deadband", segment_advance_lateral_deadband_, 0.02);
        pnh_.param("segment_rotate_min_angular_speed", segment_rotate_min_angular_speed_, 0.10);
        pnh_.param("point_realign_heading_tolerance", point_realign_heading_tolerance_, 0.26);
        pnh_.param("point_realign_lateral_tolerance", point_realign_lateral_tolerance_, 0.16);
        pnh_.param("segment_realign_heading_tolerance", segment_realign_heading_tolerance_, 0.30);
        pnh_.param("segment_realign_lateral_tolerance", segment_realign_lateral_tolerance_, 0.22);
        pnh_.param("control_mode_hold_sec", control_mode_hold_sec_, 0.25);
        pnh_.param("control_mode_switch_confirm_cycles", control_mode_switch_confirm_cycles_, 4);
        pnh_.param("linear_cmd_slew_rate", linear_cmd_slew_rate_, 0.90);
        pnh_.param("angular_cmd_slew_rate", angular_cmd_slew_rate_, 1.80);
        pnh_.param("rotate_only_heading_threshold", rotate_only_heading_threshold_, 0.35);
        pnh_.param("forward_control_heading_threshold", forward_control_heading_threshold_, 0.60);
        pnh_.param("target_loss_timeout", target_loss_timeout_, 1.0);
        pnh_.param("target_view_heading_half_angle", target_view_heading_half_angle_, 0.75);
        pnh_.param("target_distance_tie_threshold", target_distance_tie_threshold_, 0.10);
        pnh_.param("search_rotate_speed", search_rotate_speed_, 0.35);
        pnh_.param("search_brake_duration_sec", search_brake_duration_sec_, 0.25);
        pnh_.param("enable_spiral_traversal", enable_spiral_traversal_, true);
        pnh_.param("traversal_x_min", traversal_x_min_, 0.80);
        pnh_.param("traversal_x_max", traversal_x_max_, 11.00);
        pnh_.param("traversal_y_min", traversal_y_min_, -5.00);
        pnh_.param("traversal_y_max", traversal_y_max_, 5.00);
        pnh_.param("traversal_inset_step", traversal_inset_step_, 1.20);
        pnh_.param("traversal_waypoint_tolerance", traversal_waypoint_tolerance_, 0.25);
        pnh_.param("traversal_heading_tolerance", traversal_heading_tolerance_, 0.22);
        pnh_.param("traversal_linear_speed", traversal_linear_speed_, 0.22);
        pnh_.param("traversal_angular_gain", traversal_angular_gain_, 1.20);
        pnh_.param("traversal_max_angular_speed", traversal_max_angular_speed_, 0.45);
        pnh_.param("heading_deadband", heading_deadband_, 0.03);
        pnh_.param("lateral_deadband", lateral_deadband_, 0.02);
        pnh_.param("segment_heading_enter_tolerance", segment_heading_enter_tolerance_, 0.12);
        pnh_.param("segment_heading_exit_tolerance", segment_heading_exit_tolerance_, 0.24);
        pnh_.param("segment_lateral_enter_tolerance", segment_lateral_enter_tolerance_, 0.10);
        pnh_.param("segment_lateral_exit_tolerance", segment_lateral_exit_tolerance_, 0.18);
        pnh_.param("segment_pickup_band_hysteresis", segment_pickup_band_hysteresis_, 0.06);
        pnh_.param("segment_advance_max_angular_speed", segment_advance_max_angular_speed_, 0.22);
        pnh_.param("point_heading_exit_tolerance", point_heading_exit_tolerance_, 0.18);
        pnh_.param("point_lateral_exit_tolerance", point_lateral_exit_tolerance_, 0.09);
        pnh_.param("startup_delay_sec", startup_delay_sec_, 0.0);
        pnh_.param("wait_for_manual_start", wait_for_manual_start_, false);
        pnh_.param<std::string>("artifacts_output_dir", artifacts_output_dir_, "");

        if (!pnh_.getParam("ball_names", ball_names_)) {
            ball_names_ = {"tennis_ball_1", "tennis_ball_2", "tennis_ball_3", "tennis_ball_4"};
        }

        if (strategy_ != "point" && strategy_ != "segment") {
            ROS_WARN("Unknown strategy '%s', fallback to 'segment'.", strategy_.c_str());
            strategy_ = "segment";
        }

        if (planning_source_ != "model" && planning_source_ != "vision") {
            ROS_WARN("Unknown planning_source '%s', fallback to 'model'.", planning_source_.c_str());
            planning_source_ = "model";
        }

        pnh_.param("visual_target_match_distance", visual_target_match_distance_, 1.50);
        pnh_.param("visual_positions_timeout", visual_positions_timeout_, 1.00);
        pnh_.param("visual_position_ema_alpha", visual_position_ema_alpha_, 0.20);
        pnh_.param("camera_offset_x", camera_offset_x_, 0.25);
        pnh_.param("camera_offset_y", camera_offset_y_, 0.0);

        manual_start_requested_ = !wait_for_manual_start_;
    }

    void loadSceneBallAnchors() {
        if (!pnh_.hasParam("balls")) {
            return;
        }

        XmlRpc::XmlRpcValue ball_specs;
        pnh_.getParam("balls", ball_specs);
        if (ball_specs.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            return;
        }

        for (int i = 0; i < ball_specs.size(); ++i) {
            if (ball_specs[i].getType() != XmlRpc::XmlRpcValue::TypeStruct ||
                !ball_specs[i].hasMember("name")) {
                continue;
            }
            const std::string ball_name = static_cast<std::string>(ball_specs[i]["name"]);
            auto it = balls_.find(ball_name);
            if (it == balls_.end()) {
                continue;
            }

            if (ball_specs[i].hasMember("x")) {
                it->second.initial_x = static_cast<double>(ball_specs[i]["x"]);
                it->second.world_x = it->second.initial_x;
            }
            if (ball_specs[i].hasMember("y")) {
                it->second.initial_y = static_cast<double>(ball_specs[i]["y"]);
                it->second.world_y = it->second.initial_y;
            }
            it->second.initial_recorded = true;
        }
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        latest_states_ = *msg;
        have_model_states_ = true;
    }

    void stereoBallPositionsCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        latest_stereo_ball_positions_ = *msg;
        have_stereo_ball_positions_ = true;
    }

    double normalizeAngle(double angle) const {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    double yawFromQuaternion(const geometry_msgs::Quaternion& q) const {
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    bool updateWorldState(double& robot_x, double& robot_y, double& robot_yaw) {
        if (!have_model_states_) {
            return false;
        }

        for (auto& entry : balls_) {
            entry.second.model_visible = false;
        }

        bool robot_found = false;
        for (std::size_t i = 0; i < latest_states_.name.size(); ++i) {
            const std::string& name = latest_states_.name[i];
            if (name == robot_model_name_) {
                robot_x = latest_states_.pose[i].position.x;
                robot_y = latest_states_.pose[i].position.y;
                robot_yaw = yawFromQuaternion(latest_states_.pose[i].orientation);
                robot_found = true;
            }

            auto it = balls_.find(name);
            if (it != balls_.end()) {
                it->second.model_world_x = latest_states_.pose[i].position.x;
                it->second.model_world_y = latest_states_.pose[i].position.y;
                it->second.model_visible = true;
            }
        }

        return robot_found;
    }

    void updateBallStatesFromVision(double robot_x, double robot_y, double robot_yaw) {
        for (auto& entry : balls_) {
            entry.second.visible = false;
        }

        if (!have_stereo_ball_positions_) {
            return;
        }

        const double age_sec = std::abs((ros::Time::now() - latest_stereo_ball_positions_.header.stamp).toSec());
        if (age_sec > visual_positions_timeout_) {
            return;
        }

        std::vector<std::string> matched_ball_names;
        matched_ball_names.reserve(latest_stereo_ball_positions_.poses.size());
        std::size_t unmatched_visual_targets = 0;

        for (const geometry_msgs::Pose& pose : latest_stereo_ball_positions_.poses) {
            const double rel_forward = pose.position.x + camera_offset_x_;
            const double rel_lateral = pose.position.y + camera_offset_y_;
            const double est_world_x = robot_x + std::cos(robot_yaw) * rel_forward - std::sin(robot_yaw) * rel_lateral;
            const double est_world_y = robot_y + std::sin(robot_yaw) * rel_forward + std::cos(robot_yaw) * rel_lateral;

            double best_match_distance = std::numeric_limits<double>::max();
            BallInfo* best_match = nullptr;

            for (auto& entry : balls_) {
                BallInfo& ball = entry.second;
                if (ball.picked) {
                    continue;
                }
                if (std::find(matched_ball_names.begin(), matched_ball_names.end(), ball.name) != matched_ball_names.end()) {
                    continue;
                }

                const bool use_live_anchor = ball.world_estimate_initialized &&
                                             !ball.last_seen.isZero() &&
                                             (ros::Time::now() - ball.last_seen).toSec() <= target_loss_timeout_;
                const double anchor_x = use_live_anchor ? ball.world_x : (ball.initial_recorded ? ball.initial_x : ball.world_x);
                const double anchor_y = use_live_anchor ? ball.world_y : (ball.initial_recorded ? ball.initial_y : ball.world_y);
                const double match_distance = std::hypot(anchor_x - est_world_x, anchor_y - est_world_y);
                if (match_distance < best_match_distance) {
                    best_match_distance = match_distance;
                    best_match = &ball;
                }
            }

            if (best_match == nullptr || best_match_distance > visual_target_match_distance_) {
                ++unmatched_visual_targets;
                continue;
            }

            if (best_match->world_estimate_initialized) {
                best_match->world_x = visual_position_ema_alpha_ * est_world_x +
                                      (1.0 - visual_position_ema_alpha_) * best_match->world_x;
                best_match->world_y = visual_position_ema_alpha_ * est_world_y +
                                      (1.0 - visual_position_ema_alpha_) * best_match->world_y;
            } else {
                best_match->world_x = est_world_x;
                best_match->world_y = est_world_y;
                best_match->world_estimate_initialized = true;
            }
            best_match->visible = true;
            best_match->last_seen = ros::Time::now();
            matched_ball_names.push_back(best_match->name);
        }

        ROS_INFO_THROTTLE(1.0,
                          "vision_control poses=%zu matched=%zu unmatched=%zu match_thresh=%.2f ema_alpha=%.2f",
                          latest_stereo_ball_positions_.poses.size(),
                          matched_ball_names.size(),
                          unmatched_visual_targets,
                          visual_target_match_distance_,
                          visual_position_ema_alpha_);
    }

    bool selectNearestTarget(double robot_x, double robot_y, std::string& target_name) {
        double best_distance = std::numeric_limits<double>::max();
        double best_rel_lateral = 0.0;
        bool found = false;

        for (const auto& entry : balls_) {
            const BallInfo& ball = entry.second;
            if (ball.picked || !(useModelPlanning() ? ball.model_visible : ball.visible)) {
                continue;
            }

            const double dx = (useModelPlanning() ? ball.model_world_x : ball.world_x) - robot_x;
            const double dy = (useModelPlanning() ? ball.model_world_y : ball.world_y) - robot_y;
            const double distance = std::hypot(dx, dy);
            const double rel_lateral = dy;
            if (distance < (best_distance - target_distance_tie_threshold_)) {
                best_distance = distance;
                best_rel_lateral = rel_lateral;
                target_name = ball.name;
                found = true;
            } else if (std::abs(distance - best_distance) <= target_distance_tie_threshold_) {
                const bool current_is_right = rel_lateral < 0.0;
                const bool best_is_right = best_rel_lateral < 0.0;
                if (current_is_right && !best_is_right) {
                    best_distance = distance;
                    best_rel_lateral = rel_lateral;
                    target_name = ball.name;
                    found = true;
                }
            }
        }

        return found;
    }

    double clamp(double value, double min_value, double max_value) const {
        return std::max(min_value, std::min(max_value, value));
    }

    bool ballAvailable(const BallInfo& ball) const {
        if (ball.picked) {
            return false;
        }

        if (useModelPlanning()) {
            return ball.model_visible;
        }

        if (ball.visible) {
            return true;
        }

        return !ball.last_seen.isZero() && (ros::Time::now() - ball.last_seen).toSec() <= target_loss_timeout_;
    }

    bool anyAvailableTargets() const {
        for (const auto& entry : balls_) {
            if (!entry.second.picked) {
                return true;
            }
        }
        return false;
    }

    bool isBallInView(double rel_forward, double heading_abs) const {
        return rel_forward > 0.0 && heading_abs <= target_view_heading_half_angle_;
    }

    bool selectTargetByPolicy(double robot_x,
                              double robot_y,
                              double robot_yaw,
                              std::string& target_name) {
        double best_distance = std::numeric_limits<double>::max();
        double best_rel_lateral = 0.0;
        std::string best_target_name;

        for (const auto& entry : balls_) {
            const BallInfo& ball = entry.second;
            if (!ballAvailable(ball)) {
                continue;
            }

            double rel_forward = 0.0;
            double rel_lateral = 0.0;
            double heading = 0.0;
            double distance = 0.0;
            computeRelativeTarget(ball, robot_x, robot_y, robot_yaw, rel_forward, rel_lateral, heading, distance);

            if (!useModelPlanning() && !isBallInView(rel_forward, std::abs(heading))) {
                continue;
            }

            if (distance < (best_distance - target_distance_tie_threshold_)) {
                best_distance = distance;
                best_rel_lateral = rel_lateral;
                best_target_name = ball.name;
            } else if (std::abs(distance - best_distance) <= target_distance_tie_threshold_) {
                const bool current_is_right = rel_lateral < 0.0;
                const bool best_is_right = best_rel_lateral < 0.0;
                if (current_is_right && !best_is_right) {
                    best_distance = distance;
                    best_rel_lateral = rel_lateral;
                    best_target_name = ball.name;
                }
            }
        }

        if (!best_target_name.empty()) {
            target_name = best_target_name;
            return true;
        }

        return false;
    }

    bool selectTargetWithLock(double robot_x,
                              double robot_y,
                              double robot_yaw,
                              std::string& target_name) {
        (void)robot_x;
        (void)robot_y;
        (void)robot_yaw;

        if (!current_target_name_.empty()) {
            auto current_it = balls_.find(current_target_name_);
            if (current_it != balls_.end() && ballAvailable(current_it->second)) {
                target_name = current_target_name_;
                return true;
            }
            current_target_name_.clear();
            setControlMode(ControlMode::ROTATE_ONLY);
        }

        std::string policy_target_name;
        const bool have_policy_target = selectTargetByPolicy(robot_x, robot_y, robot_yaw, policy_target_name);

        if (have_policy_target) {
            current_target_name_ = policy_target_name;
            setControlMode(ControlMode::ROTATE_ONLY);
            target_name = current_target_name_;
            return true;
        }

        return false;
    }

    void resetSearchTracking() {
        search_active_ = false;
        search_accumulated_yaw_rad_ = 0.0;
        search_last_yaw_rad_ = 0.0;
        search_has_last_yaw_sample_ = false;
        search_brake_end_time_ = ros::Time();
    }

    void resetTraversalTracking() {
        traversal_active_ = false;
        traversal_waypoints_.clear();
        traversal_waypoint_index_ = 0;
    }

    void beginSearch(double robot_yaw) {
        if (search_active_) {
            return;
        }

        // Hard-stop before search so the next command is a pure in-place
        // rotation about the robot center.
        stopRobot();
        search_active_ = true;
        search_accumulated_yaw_rad_ = 0.0;
        search_last_yaw_rad_ = robot_yaw;
        search_has_last_yaw_sample_ = true;
        search_brake_end_time_ = ros::Time::now() + ros::Duration(search_brake_duration_sec_);
        current_target_name_.clear();
        setControlMode(ControlMode::ROTATE_ONLY);
    }

    bool isSearchBrakeActive() const {
        return search_active_ && !search_brake_end_time_.isZero() && ros::Time::now() < search_brake_end_time_;
    }

    bool updateSearchProgress(double robot_yaw) {
        if (!search_active_) {
            return false;
        }

        if (search_has_last_yaw_sample_) {
            search_accumulated_yaw_rad_ += std::abs(normalizeAngle(robot_yaw - search_last_yaw_rad_));
        } else {
            search_has_last_yaw_sample_ = true;
        }
        search_last_yaw_rad_ = robot_yaw;
        return search_accumulated_yaw_rad_ >= (2.0 * M_PI);
    }

    void publishSearchRotation() {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = search_rotate_speed_;
        last_linear_cmd_ = 0.0;
        last_angular_cmd_ = cmd.angular.z;
        cmd_pub_.publish(cmd);
    }

    void buildSpiralTraversalWaypoints() {
        traversal_waypoints_.clear();

        double left = traversal_x_min_;
        double right = traversal_x_max_;
        double bottom = traversal_y_min_;
        double top = traversal_y_max_;
        const double step = std::max(0.20, traversal_inset_step_);

        while (left <= right && bottom <= top) {
            traversal_waypoints_.emplace_back(left, top);
            traversal_waypoints_.emplace_back(right, top);
            traversal_waypoints_.emplace_back(right, bottom);
            traversal_waypoints_.emplace_back(left, bottom);
            left += step;
            right -= step;
            bottom += step;
            top -= step;
        }
    }

    void beginSpiralTraversal() {
        buildSpiralTraversalWaypoints();
        traversal_waypoint_index_ = 0;
        traversal_active_ = !traversal_waypoints_.empty();
        current_target_name_.clear();
        setControlMode(ControlMode::ROTATE_ONLY);
        if (traversal_active_) {
            ROS_INFO("Search rotation completed without finding a ball. Entering half-court spiral traversal mode with %zu waypoints.",
                     traversal_waypoints_.size());
        }
    }

    bool updateTraversalWaypoint(double robot_x, double robot_y) {
        if (!traversal_active_) {
            return false;
        }

        while (traversal_waypoint_index_ < traversal_waypoints_.size()) {
            const auto& waypoint = traversal_waypoints_[traversal_waypoint_index_];
            const double distance = std::hypot(waypoint.first - robot_x, waypoint.second - robot_y);
            if (distance > traversal_waypoint_tolerance_) {
                return true;
            }
            ++traversal_waypoint_index_;
        }

        traversal_active_ = false;
        return false;
    }

    void publishTraversalControl(double robot_x, double robot_y, double robot_yaw) {
        if (!traversal_active_ || traversal_waypoint_index_ >= traversal_waypoints_.size()) {
            stopRobot();
            return;
        }

        const auto& waypoint = traversal_waypoints_[traversal_waypoint_index_];
        const double dx = waypoint.first - robot_x;
        const double dy = waypoint.second - robot_y;
        const double rel_forward = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
        const double rel_lateral = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
        const double heading = std::atan2(rel_lateral, rel_forward);
        const double heading_abs = std::abs(heading);

        geometry_msgs::Twist cmd;
        if (rel_forward < 0.0 || heading_abs > traversal_heading_tolerance_) {
            cmd.linear.x = 0.0;
            const double requested_angular = traversal_angular_gain_ * heading;
            const double angular_sign = (requested_angular >= 0.0) ? 1.0 : -1.0;
            const double angular_mag = std::max(shared_rotate_min_angular_speed_, std::abs(requested_angular));
            cmd.angular.z = angular_sign * clamp(angular_mag, 0.0, traversal_max_angular_speed_);
            last_linear_cmd_ = 0.0;
            last_angular_cmd_ = cmd.angular.z;
            cmd_pub_.publish(cmd);
            return;
        }

        cmd.linear.x = traversal_linear_speed_;
        cmd.angular.z = clamp(traversal_angular_gain_ * heading,
                              -traversal_max_angular_speed_,
                              traversal_max_angular_speed_);
        cmd_pub_.publish(smoothCommand(cmd));
    }

    bool computeRelativeTarget(const BallInfo& ball,
                               double robot_x,
                               double robot_y,
                               double robot_yaw,
                               double& rel_forward,
                               double& rel_lateral,
                               double& heading,
                               double& distance) const {
        if (useModelPlanning()) {
            return computeRelativeTargetFromModel(ball,
                                                  robot_x,
                                                  robot_y,
                                                  robot_yaw,
                                                  rel_forward,
                                                  rel_lateral,
                                                  heading,
                                                  distance);
        }

        const double dx = ball.world_x - robot_x;
        const double dy = ball.world_y - robot_y;

        rel_forward = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
        rel_lateral = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
        heading = std::atan2(rel_lateral, rel_forward);
        distance = std::hypot(rel_forward, rel_lateral);
        return true;
    }

    bool computeRelativeTargetFromModel(const BallInfo& ball,
                                        double robot_x,
                                        double robot_y,
                                        double robot_yaw,
                                        double& rel_forward,
                                        double& rel_lateral,
                                        double& heading,
                                        double& distance) const {
        if (!ball.model_visible) {
            return false;
        }

        const double dx = ball.model_world_x - robot_x;
        const double dy = ball.model_world_y - robot_y;

        rel_forward = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
        rel_lateral = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
        heading = std::atan2(rel_lateral, rel_forward);
        distance = std::hypot(rel_forward, rel_lateral);
        return true;
    }

    bool isPickupSuccess(bool point_strategy,
                         double rel_forward,
                         double rel_lateral,
                         double heading) const {
        (void)point_strategy;
        (void)heading;
        const double effective_depth_tolerance = ball_radius_ + pickup_tolerance_ + depth_error_margin_;

        return std::abs(rel_lateral) <= segmentPickupBandHalfWidth() &&
               rel_forward <= front_boundary_x_ &&
               rel_forward >= (front_boundary_x_ - effective_depth_tolerance);
    }

    double segmentPickupBandHalfWidth() const {
        return std::max(0.0, pickup_half_width_ - segment_pickup_side_shrink_);
    }

    double segmentGuidanceBandHalfWidth() const {
        return std::max(0.0, segmentPickupBandHalfWidth() - segment_guidance_inset_);
    }

    double computeBandControlHeading(double rel_forward,
                                     double rel_lateral,
                                     double band_half_width) const {
        if (rel_forward <= 0.0) {
            return std::atan2(rel_lateral, rel_forward);
        }

        const double lateral_abs = std::abs(rel_lateral);
        if (lateral_abs <= band_half_width) {
            return 0.0;
        }

        const double distance = std::hypot(rel_forward, rel_lateral);
        const double desired_bearing_abs = std::asin(clamp(band_half_width / std::max(distance, 1e-6), 0.0, 1.0));
        const double desired_bearing = std::copysign(desired_bearing_abs, rel_lateral);
        return normalizeAngle(std::atan2(rel_lateral, rel_forward) - desired_bearing);
    }

    double strategyControlBandHalfWidth(bool point_strategy) const {
        return point_strategy ? 0.0 : segmentGuidanceBandHalfWidth();
    }

    double computeStrategyLateralOverflow(bool point_strategy, double rel_lateral) const {
        return std::max(0.0, std::abs(rel_lateral) - strategyControlBandHalfWidth(point_strategy));
    }

    double computePointControlHeading(double rel_forward, double rel_lateral) const {
        return computeBandControlHeading(rel_forward, rel_lateral, 0.0);
    }

    double computeSegmentControlHeading(double rel_forward, double rel_lateral) const {
        return computeBandControlHeading(rel_forward, rel_lateral, segmentGuidanceBandHalfWidth());
    }

    double computeStrategyControlHeading(bool point_strategy,
                                         double rel_forward,
                                         double rel_lateral) const {
        return point_strategy
            ? computePointControlHeading(rel_forward, rel_lateral)
            : computeSegmentControlHeading(rel_forward, rel_lateral);
    }

    bool isStrategyAdvanceReady(bool point_strategy,
                                double rel_forward,
                                double rel_lateral,
                                double control_heading_abs) const {
        if (rel_forward <= 0.0) {
            return false;
        }

        return computeStrategyLateralOverflow(point_strategy, rel_lateral) <= shared_advance_lateral_overflow_tolerance_ &&
               control_heading_abs <= shared_advance_heading_tolerance_;
    }

    bool needStrategyRealign(bool point_strategy,
                             double rel_forward,
                             double rel_lateral,
                             double control_heading_abs) const {
        if (rel_forward <= 0.0) {
            return true;
        }

        const double forward_error = std::max(0.0, rel_forward - front_boundary_x_);
        if (forward_error <= shared_realign_forward_error_threshold_) {
            return false;
        }

        return computeStrategyLateralOverflow(point_strategy, rel_lateral) > shared_realign_lateral_overflow_tolerance_ ||
               control_heading_abs > shared_realign_heading_tolerance_;
    }

    double pointProgressScore(double rel_lateral, double heading) const {
        return 1.4 * std::abs(heading) + 1.0 * std::abs(rel_lateral);
    }

    void resetPointRecoveryTracking() {
        point_progress_initialized_ = false;
        point_recovery_active_ = false;
        point_recovery_end_time_ = ros::Time();
    }

    void beginPointRecovery(double rel_lateral) {
        point_recovery_active_ = true;
        point_recovery_end_time_ = ros::Time::now() + ros::Duration(point_recovery_duration_sec_);

        if (std::abs(rel_lateral) > 1e-3) {
            point_recovery_turn_sign_ = (rel_lateral >= 0.0) ? -1.0 : 1.0;
        } else {
            point_recovery_turn_sign_ *= -1.0;
        }

        ++point_recovery_count_;
        current_target_name_.clear();
        setControlMode(ControlMode::ROTATE_ONLY);
        point_progress_initialized_ = false;

        ROS_WARN("Point strategy stuck detected, applying short forward recovery.");
    }

    bool handlePointRecoveryIfActive() {
        if (!point_recovery_active_) {
            return false;
        }

        if (ros::Time::now() >= point_recovery_end_time_) {
            point_recovery_active_ = false;
            point_progress_initialized_ = false;
            stopRobot();
            return false;
        }

        geometry_msgs::Twist cmd;
        cmd.linear.x = point_recovery_forward_speed_;
        cmd.angular.z = point_recovery_turn_sign_ * point_recovery_turn_speed_;
        cmd_pub_.publish(cmd);
        return true;
    }

    void updatePointProgress(const std::string& target_name,
                             double rel_lateral,
                             double heading) {
        const double score = pointProgressScore(rel_lateral, heading);

        if (!point_progress_initialized_ || point_progress_target_name_ != target_name) {
            point_progress_initialized_ = true;
            point_progress_target_name_ = target_name;
            point_best_progress_score_ = score;
            point_last_progress_time_ = ros::Time::now();
            return;
        }

        if (score < (point_best_progress_score_ - point_progress_score_epsilon_)) {
            point_best_progress_score_ = score;
            point_last_progress_time_ = ros::Time::now();
        }
    }

    bool isBallWithinPickupBand(bool point_strategy,
                                double rel_forward,
                                double rel_lateral) const {
        (void)point_strategy;
        const double effective_depth_tolerance = ball_radius_ + pickup_tolerance_ + depth_error_margin_;

        return std::abs(rel_lateral) <= segmentPickupBandHalfWidth() &&
               rel_forward <= front_boundary_x_ &&
               rel_forward >= (front_boundary_x_ - effective_depth_tolerance);
    }

    bool pickupBallsInContactBand(double robot_x, double robot_y, double robot_yaw) {
        const bool point_strategy = isPointStrategy();
        std::vector<std::string> touched_balls;

        for (const std::string& ball_name : ball_names_) {
            auto it = balls_.find(ball_name);
            if (it == balls_.end()) {
                continue;
            }

            const BallInfo& ball = it->second;
            if (ball.picked || !ball.model_visible) {
                continue;
            }

            double rel_forward = 0.0;
            double rel_lateral = 0.0;
            double heading = 0.0;
            double distance = 0.0;
            computeRelativeTargetFromModel(ball, robot_x, robot_y, robot_yaw, rel_forward, rel_lateral, heading, distance);

            if (isBallWithinPickupBand(point_strategy, rel_forward, rel_lateral)) {
                touched_balls.push_back(ball.name);
            }
        }

        for (const std::string& ball_name : touched_balls) {
            removePickedBall(ball_name);
        }

        if (!touched_balls.empty()) {
            ROS_INFO("Picked %zu ball(s) from contact band.", touched_balls.size());
            return true;
        }

        return false;
    }

    void removePickedBall(const std::string& ball_name) {
        auto it = balls_.find(ball_name);
        if (it == balls_.end() || it->second.picked) {
            return;
        }

        it->second.picked = true;
        it->second.pickup_time_sec = currentRunTimeSec();
        it->second.pickup_order = static_cast<int>(picked_ball_count_ + 1);
        ++picked_ball_count_;
        if (picked_ball_count_ == ball_names_.size()) {
            markRunFinishedIfNeeded();
            publishPickupComplete(true);
        }
        if (current_target_name_ == ball_name) {
            current_target_name_.clear();
            setControlMode(ControlMode::ROTATE_ONLY);
        }
        resetPointRecoveryTracking();
        resetSearchTracking();
        resetTraversalTracking();

        gazebo_msgs::DeleteModel delete_srv;
        delete_srv.request.model_name = ball_name;
        if (delete_model_client_.exists() && delete_model_client_.call(delete_srv) && delete_srv.response.success) {
            ROS_INFO("Picked ball removed from Gazebo: %s", ball_name.c_str());
        } else {
            ROS_WARN("Picked ball marked logically only: %s", ball_name.c_str());
        }
    }

    void stopRobot() {
        geometry_msgs::Twist cmd;
        last_linear_cmd_ = 0.0;
        last_angular_cmd_ = 0.0;
        cmd_pub_.publish(cmd);
    }

    void publishPickupComplete(bool complete) {
        std_msgs::Bool msg;
        msg.data = complete;
        pickup_complete_pub_.publish(msg);
    }

    void setControlMode(ControlMode new_mode) {
        if (control_mode_ == new_mode) {
            return;
        }

        control_mode_ = new_mode;
        control_mode_last_switch_time_ = ros::Time::now();
        pending_mode_switch_valid_ = false;
        pending_mode_switch_count_ = 0;
    }

    bool canSwitchControlMode() const {
        if (control_mode_last_switch_time_.isZero()) {
            return true;
        }

        return (ros::Time::now() - control_mode_last_switch_time_).toSec() >= control_mode_hold_sec_;
    }

    void clearPendingModeSwitch() {
        pending_mode_switch_valid_ = false;
        pending_mode_switch_count_ = 0;
    }

    void requestControlModeSwitch(ControlMode desired_mode) {
        if (desired_mode == control_mode_) {
            clearPendingModeSwitch();
            return;
        }

        if (pending_mode_switch_valid_ && pending_control_mode_ == desired_mode) {
            ++pending_mode_switch_count_;
        } else {
            pending_mode_switch_valid_ = true;
            pending_control_mode_ = desired_mode;
            pending_mode_switch_count_ = 1;
        }

        if (pending_mode_switch_count_ >= control_mode_switch_confirm_cycles_) {
            setControlMode(desired_mode);
        }
    }

    geometry_msgs::Twist smoothCommand(const geometry_msgs::Twist& target_cmd) {
        geometry_msgs::Twist smoothed_cmd = target_cmd;
        const double dt = std::max(1e-3, 1.0 / std::max(1.0, control_frequency_));
        const double max_linear_delta = linear_cmd_slew_rate_ * dt;
        const double max_angular_delta = angular_cmd_slew_rate_ * dt;

        smoothed_cmd.linear.x = clamp(target_cmd.linear.x,
                                      last_linear_cmd_ - max_linear_delta,
                                      last_linear_cmd_ + max_linear_delta);
        smoothed_cmd.angular.z = clamp(target_cmd.angular.z,
                                       last_angular_cmd_ - max_angular_delta,
                                       last_angular_cmd_ + max_angular_delta);

        last_linear_cmd_ = smoothed_cmd.linear.x;
        last_angular_cmd_ = smoothed_cmd.angular.z;
        return smoothed_cmd;
    }

    bool handleStartRequest(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& response) {
        if (manual_start_requested_) {
            response.success = true;
            response.message = "ball_pickup_controller already started";
            return true;
        }

        manual_start_requested_ = true;
        startup_reference_time_ = ros::Time::now();
        startup_time_initialized_ = true;
        current_target_name_.clear();
        setControlMode(ControlMode::ROTATE_ONLY);
        resetPointRecoveryTracking();
        resetSearchTracking();
        resetTraversalTracking();
        publishPickupComplete(false);

        response.success = true;
        response.message = "ball_pickup_controller start accepted";
        ROS_INFO("ball_pickup_controller received manual start request.");
        return true;
    }

    double currentRunTimeSec() const {
        if (!run_started_ || run_start_time_.isZero()) {
            return 0.0;
        }
        return std::max(0.0, (ros::Time::now() - run_start_time_).toSec());
    }

    void markRunStartedIfNeeded() {
        if (!run_started_) {
            run_started_ = true;
            run_start_time_ = ros::Time::now();
            ROS_INFO("ball_pickup_controller run started.");
        }
    }

    void recordPathSample(double robot_x, double robot_y, double robot_yaw) {
        markRunStartedIfNeeded();

        PathSample sample;
        sample.sim_time_sec = currentRunTimeSec();
        sample.x = robot_x;
        sample.y = robot_y;
        sample.yaw = robot_yaw;

        if (!path_samples_.empty()) {
            const PathSample& last = path_samples_.back();
            total_path_length_m_ += std::hypot(sample.x - last.x, sample.y - last.y);
            total_abs_yaw_change_rad_ += std::abs(normalizeAngle(sample.yaw - last.yaw));
        }

        path_samples_.push_back(sample);
    }

    void markRunFinishedIfNeeded() {
        if (run_started_ && !run_finished_) {
            run_finished_ = true;
            run_end_time_ = ros::Time::now();
        }
    }

    std::string formatDouble(double value, int precision = 3) const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << value;
        return oss.str();
    }

    std::pair<double, double> projectPoint(double x,
                                           double y,
                                           double min_x,
                                           double max_x,
                                           double min_y,
                                           double max_y,
                                           double width,
                                           double height,
                                           double margin) const {
        const double usable_width = width - 2.0 * margin;
        const double usable_height = height - 2.0 * margin;
        const double x_norm = (x - min_x) / std::max(1e-6, max_x - min_x);
        const double y_norm = (y - min_y) / std::max(1e-6, max_y - min_y);
        const double px = margin + x_norm * usable_width;
        const double py = height - (margin + y_norm * usable_height);
        return {px, py};
    }

    void exportPathCsv() const {
        const std::string csv_path = artifacts_output_dir_ + "/path.csv";
        std::ofstream ofs(csv_path);
        if (!ofs) {
            ROS_WARN("Failed to write path csv: %s", csv_path.c_str());
            return;
        }

        ofs << "sim_time_sec,x_m,y_m,yaw_rad\n";
        for (const PathSample& sample : path_samples_) {
            ofs << formatDouble(sample.sim_time_sec, 4) << ","
                << formatDouble(sample.x, 4) << ","
                << formatDouble(sample.y, 4) << ","
                << formatDouble(sample.yaw, 4) << "\n";
        }
    }

    void exportPathSvg() const {
        if (path_samples_.empty()) {
            return;
        }

        const std::string svg_path = artifacts_output_dir_ + "/path.svg";
        std::ofstream ofs(svg_path);
        if (!ofs) {
            ROS_WARN("Failed to write path svg: %s", svg_path.c_str());
            return;
        }

        constexpr double width = 1000.0;
        constexpr double height = 700.0;
        constexpr double margin = 70.0;

        double min_x = path_samples_.front().x;
        double max_x = path_samples_.front().x;
        double min_y = path_samples_.front().y;
        double max_y = path_samples_.front().y;

        for (const PathSample& sample : path_samples_) {
            min_x = std::min(min_x, sample.x);
            max_x = std::max(max_x, sample.x);
            min_y = std::min(min_y, sample.y);
            max_y = std::max(max_y, sample.y);
        }

        for (const auto& entry : balls_) {
            const BallInfo& ball = entry.second;
            if (!ball.initial_recorded) {
                continue;
            }
            min_x = std::min(min_x, ball.initial_x);
            max_x = std::max(max_x, ball.initial_x);
            min_y = std::min(min_y, ball.initial_y);
            max_y = std::max(max_y, ball.initial_y);
        }

        min_x -= 0.5;
        max_x += 0.5;
        min_y -= 0.5;
        max_y += 0.5;

        if ((max_x - min_x) < 2.0) {
            min_x -= 1.0;
            max_x += 1.0;
        }
        if ((max_y - min_y) < 2.0) {
            min_y -= 1.0;
            max_y += 1.0;
        }

        ofs << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
            << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << " " << height << "\">\n";
        ofs << "  <rect x=\"0\" y=\"0\" width=\"" << width << "\" height=\"" << height
            << "\" fill=\"#ffffff\"/>\n";
        ofs << "  <rect x=\"" << margin << "\" y=\"" << margin << "\" width=\"" << (width - 2 * margin)
            << "\" height=\"" << (height - 2 * margin) << "\" fill=\"#f8fbff\" stroke=\"#c8d4e3\" stroke-width=\"2\"/>\n";

        for (int i = 0; i <= 5; ++i) {
            const double gx = margin + i * (width - 2 * margin) / 5.0;
            const double gy = margin + i * (height - 2 * margin) / 5.0;
            ofs << "  <line x1=\"" << gx << "\" y1=\"" << margin << "\" x2=\"" << gx << "\" y2=\""
                << (height - margin) << "\" stroke=\"#e6edf5\" stroke-width=\"1\"/>\n";
            ofs << "  <line x1=\"" << margin << "\" y1=\"" << gy << "\" x2=\"" << (width - margin)
                << "\" y2=\"" << gy << "\" stroke=\"#e6edf5\" stroke-width=\"1\"/>\n";
        }

        ofs << "  <polyline fill=\"none\" stroke=\"#1f77b4\" stroke-width=\"4\" points=\"";
        for (const PathSample& sample : path_samples_) {
            const auto pt = projectPoint(sample.x, sample.y, min_x, max_x, min_y, max_y, width, height, margin);
            ofs << formatDouble(pt.first, 2) << "," << formatDouble(pt.second, 2) << " ";
        }
        ofs << "\"/>\n";

        const auto start_pt = projectPoint(path_samples_.front().x, path_samples_.front().y,
                                           min_x, max_x, min_y, max_y, width, height, margin);
        const auto end_pt = projectPoint(path_samples_.back().x, path_samples_.back().y,
                                         min_x, max_x, min_y, max_y, width, height, margin);
        ofs << "  <circle cx=\"" << formatDouble(start_pt.first, 2) << "\" cy=\"" << formatDouble(start_pt.second, 2)
            << "\" r=\"7\" fill=\"#2ca02c\"/>\n";
        ofs << "  <circle cx=\"" << formatDouble(end_pt.first, 2) << "\" cy=\"" << formatDouble(end_pt.second, 2)
            << "\" r=\"7\" fill=\"#d62728\"/>\n";
        ofs << "  <text x=\"" << formatDouble(start_pt.first + 10, 2) << "\" y=\"" << formatDouble(start_pt.second - 10, 2)
            << "\" font-size=\"18\" fill=\"#2ca02c\">start</text>\n";
        ofs << "  <text x=\"" << formatDouble(end_pt.first + 10, 2) << "\" y=\"" << formatDouble(end_pt.second - 10, 2)
            << "\" font-size=\"18\" fill=\"#d62728\">end</text>\n";

        for (const auto& entry : balls_) {
            const BallInfo& ball = entry.second;
            if (!ball.initial_recorded) {
                continue;
            }
            const auto pt = projectPoint(ball.initial_x, ball.initial_y, min_x, max_x, min_y, max_y, width, height, margin);
            const std::string fill = ball.picked ? "#ffcc00" : "#888888";
            ofs << "  <circle cx=\"" << formatDouble(pt.first, 2) << "\" cy=\"" << formatDouble(pt.second, 2)
                << "\" r=\"8\" fill=\"" << fill << "\" stroke=\"#333333\" stroke-width=\"1.5\"/>\n";
            std::ostringstream label;
            label << ball.name;
            if (ball.pickup_order > 0) {
                label << " #" << ball.pickup_order;
            }
            ofs << "  <text x=\"" << formatDouble(pt.first + 12, 2) << "\" y=\"" << formatDouble(pt.second - 12, 2)
                << "\" font-size=\"16\" fill=\"#333333\">" << label.str() << "</text>\n";
        }

        ofs << "  <text x=\"" << margin << "\" y=\"35\" font-size=\"24\" fill=\"#203040\">Robot pickup path (top view)</text>\n";
        ofs << "  <text x=\"" << margin << "\" y=\"" << (height - 18)
            << "\" font-size=\"16\" fill=\"#506070\">x range: " << formatDouble(min_x, 2) << " to "
            << formatDouble(max_x, 2) << " m, y range: " << formatDouble(min_y, 2) << " to "
            << formatDouble(max_y, 2) << " m</text>\n";
        ofs << "</svg>\n";
    }

    void exportMetricsMarkdown() const {
        const std::string metrics_path = artifacts_output_dir_ + "/metrics.md";
        std::ofstream ofs(metrics_path);
        if (!ofs) {
            ROS_WARN("Failed to write metrics markdown: %s", metrics_path.c_str());
            return;
        }

        const double duration_sec = run_started_
            ? ((run_finished_ ? run_end_time_ : ros::Time::now()) - run_start_time_).toSec()
            : 0.0;
        const double avg_speed = (duration_sec > 1e-6) ? (total_path_length_m_ / duration_sec) : 0.0;
        const double avg_abs_yaw_rate = (duration_sec > 1e-6) ? (total_abs_yaw_change_rad_ / duration_sec) : 0.0;
        const bool all_picked = (picked_ball_count_ == ball_names_.size());

        ofs << "# Pickup Experiment Summary\n\n";
        ofs << "- Strategy: `" << strategy_ << "`\n";
        ofs << "- Strategy label: `" << strategyDisplayName() << "`\n";
        ofs << "- Robot model: `" << robot_model_name_ << "`\n";
        ofs << "- Balls configured: `" << ball_names_.size() << "`\n";
        ofs << "- Balls picked: `" << picked_ball_count_ << "`\n";
        ofs << "- Run completed all pickups: `" << (all_picked ? "yes" : "no") << "`\n";
        ofs << "- Total pickup time (sim): `" << formatDouble(duration_sec, 3) << " s`\n";
        ofs << "- Total traveled distance: `" << formatDouble(total_path_length_m_, 3) << " m`\n";
        ofs << "- Average path speed: `" << formatDouble(avg_speed, 3) << " m/s`\n";
        ofs << "- Total accumulated yaw change: `" << formatDouble(total_abs_yaw_change_rad_, 3) << " rad`\n";
        ofs << "- Average absolute yaw rate: `" << formatDouble(avg_abs_yaw_rate, 3) << " rad/s`\n";
        ofs << "- Path samples: `" << path_samples_.size() << "`\n";
        ofs << "- Point recovery count: `" << point_recovery_count_ << "`\n\n";

        if (!path_samples_.empty()) {
            const PathSample& first = path_samples_.front();
            const PathSample& last = path_samples_.back();
            ofs << "## Robot Pose\n\n";
            ofs << "- Start pose: `(" << formatDouble(first.x, 3) << ", " << formatDouble(first.y, 3)
                << "), yaw=" << formatDouble(first.yaw, 3) << " rad`\n";
            ofs << "- End pose: `(" << formatDouble(last.x, 3) << ", " << formatDouble(last.y, 3)
                << "), yaw=" << formatDouble(last.yaw, 3) << " rad`\n\n";
        }

        ofs << "## Ball Pickup Timeline\n\n";
        ofs << "| Ball | Initial position (m) | Picked | Pickup order | Pickup time (s) |\n";
        ofs << "| --- | --- | --- | --- | --- |\n";
        for (const std::string& ball_name : ball_names_) {
            const BallInfo& ball = balls_.at(ball_name);
            std::ostringstream pos;
            if (ball.initial_recorded) {
                pos << "(" << formatDouble(ball.initial_x, 3) << ", " << formatDouble(ball.initial_y, 3) << ")";
            } else {
                pos << "n/a";
            }
            ofs << "| `" << ball.name << "` | " << pos.str()
                << " | " << (ball.picked ? "yes" : "no")
                << " | " << (ball.pickup_order > 0 ? std::to_string(ball.pickup_order) : "-")
                << " | " << (ball.pickup_time_sec >= 0.0 ? formatDouble(ball.pickup_time_sec, 3) : "-")
                << " |\n";
        }

        ofs << "\n## Generated Files\n\n";
        ofs << "- `screen.mp4`\n";
        ofs << "- `path.csv`\n";
        ofs << "- `path.svg`\n";
        ofs << "- `metrics.md`\n";
    }

    void exportArtifacts() {
        if (artifacts_written_ || artifacts_output_dir_.empty() || !run_started_) {
            return;
        }

        if (!run_finished_) {
            run_end_time_ = ros::Time::now();
        }

        exportPathCsv();
        exportPathSvg();
        exportMetricsMarkdown();
        artifacts_written_ = true;
        ROS_INFO("ball_pickup_controller exported artifacts to %s", artifacts_output_dir_.c_str());
    }

    const char* controlModeName() const {
        return (control_mode_ == ControlMode::ROTATE_ONLY) ? "ROTATE_ONLY" : "ADVANCE";
    }

    const char* behaviorStateName() const {
        switch (behavior_state_) {
            case BehaviorState::WAITING_START:
                return "WAITING_START";
            case BehaviorState::WAITING_DELAY:
                return "WAITING_DELAY";
            case BehaviorState::SEARCHING:
                return "SEARCHING";
            case BehaviorState::TRAVERSING:
                return "TRAVERSING";
            case BehaviorState::TRACKING:
                return "TRACKING";
            case BehaviorState::PICKUP_SUCCESS:
                return "PICKUP_SUCCESS";
            case BehaviorState::STOPPED_NO_TARGET:
                return "STOPPED_NO_TARGET";
        }
        return "UNKNOWN";
    }

    void updateControlMode(bool point_strategy,
                           double rel_forward,
                           double rel_lateral,
                           double control_heading_abs,
                           double lateral_abs) {
        (void)lateral_abs;
        const bool allow_mode_switch = canSwitchControlMode();

        if (control_mode_ == ControlMode::ROTATE_ONLY) {
            if (allow_mode_switch &&
                isStrategyAdvanceReady(point_strategy, rel_forward, rel_lateral, control_heading_abs)) {
                setControlMode(ControlMode::ADVANCE);
            } else {
                clearPendingModeSwitch();
            }
        } else {
            const bool need_realign = needStrategyRealign(point_strategy,
                                                          rel_forward,
                                                          rel_lateral,
                                                          control_heading_abs);
            if (allow_mode_switch && need_realign) {
                requestControlModeSwitch(ControlMode::ROTATE_ONLY);
            } else {
                clearPendingModeSwitch();
            }
        }
    }

    void publishControl(double rel_forward, double rel_lateral, double heading, double distance) {
        geometry_msgs::Twist cmd;

        const bool point_strategy = isPointStrategy();
        const double control_heading = computeStrategyControlHeading(point_strategy, rel_forward, rel_lateral);
        const double control_heading_abs = std::abs(control_heading);
        const bool advance_ready = isStrategyAdvanceReady(point_strategy,
                                                          rel_forward,
                                                          rel_lateral,
                                                          control_heading_abs);
        const double lateral_overflow = computeStrategyLateralOverflow(point_strategy, rel_lateral);
        const double forward_error = std::max(0.0, rel_forward - front_boundary_x_);

        updateControlMode(point_strategy, rel_forward, rel_lateral, control_heading_abs, std::abs(rel_lateral));

        double heading_cmd = 0.0;
        if (control_mode_ == ControlMode::ROTATE_ONLY) {
            heading_cmd = advance_ready ? 0.0 : control_heading;
        } else {
            heading_cmd = (control_heading_abs > shared_advance_heading_deadband_) ? control_heading : 0.0;
        }

        if (control_mode_ == ControlMode::ROTATE_ONLY) {
            cmd.linear.x = 0.0;
            if (!advance_ready && control_heading_abs > 1e-4) {
                const double requested_angular = heading_gain_ * heading_cmd;
                const double angular_sign = (requested_angular >= 0.0) ? 1.0 : -1.0;
                const double angular_mag = std::max(shared_rotate_min_angular_speed_, std::abs(requested_angular));
                cmd.angular.z = angular_sign * clamp(angular_mag, 0.0, max_angular_speed_);
            } else {
                cmd.angular.z = 0.0;
            }
        } else {
            double linear_cmd = distance_gain_ * forward_error;
            linear_cmd = clamp(linear_cmd, 0.0, max_linear_speed_);
            const double lateral_scale = (lateral_overflow <= shared_advance_lateral_deadband_)
                ? 1.0
                : clamp(1.0 - lateral_overflow / std::max(0.20, pickup_half_width_), 0.35, 1.0);
            linear_cmd *= lateral_scale;
            linear_cmd = std::max(shared_advance_min_linear_speed_, linear_cmd);

            if (forward_error < 0.10) {
                linear_cmd = std::min(linear_cmd, 0.10);
            } else if (forward_error < 0.20) {
                linear_cmd = std::min(linear_cmd, 0.18);
            }

            cmd.linear.x = linear_cmd;
            const bool allow_forward_steer =
                (control_heading_abs > shared_advance_heading_deadband_ &&
                 lateral_overflow > shared_advance_lateral_deadband_);
            const double forward_heading_cmd = allow_forward_steer ? control_heading : 0.0;
            const double shared_heading_gain = (forward_error < 0.50) ? (0.12 * heading_gain_) : (0.20 * heading_gain_);
            if (allow_forward_steer && std::abs(forward_heading_cmd) > 1e-4) {
                const double requested_angular = shared_heading_gain * forward_heading_cmd;
                const double angular_sign = (requested_angular >= 0.0) ? 1.0 : -1.0;
                const double min_angular_speed = 0.0;
                const double angular_mag = std::max(min_angular_speed, std::abs(requested_angular));
                cmd.angular.z = angular_sign * clamp(angular_mag,
                                                     0.0,
                                                     shared_advance_max_angular_speed_);
            } else {
                cmd.angular.z = 0.0;
            }
        }

        if (distance < front_boundary_x_) {
            cmd.linear.x = 0.0;
        }

        if (control_mode_ == ControlMode::ROTATE_ONLY) {
            // Any rotate-only behavior must be a pure in-place turn: no
            // residual forward smoothing is allowed.
            cmd.linear.x = 0.0;
            last_linear_cmd_ = 0.0;
            last_angular_cmd_ = cmd.angular.z;
            cmd_pub_.publish(cmd);
        } else {
            cmd_pub_.publish(smoothCommand(cmd));
        }
    }

    void controlTimerCallback(const ros::TimerEvent&) {
        if (wait_for_manual_start_ && !manual_start_requested_) {
            behavior_state_ = BehaviorState::WAITING_START;
            stopRobot();
            ROS_INFO_THROTTLE(1.0, "state=%s strategy=%s waiting for manual start request.",
                              behaviorStateName(), strategy_.c_str());
            return;
        }

        if (!startup_time_initialized_) {
            startup_reference_time_ = ros::Time::now();
            startup_time_initialized_ = true;
        }

        if ((ros::Time::now() - startup_reference_time_).toSec() < startup_delay_sec_) {
            behavior_state_ = BehaviorState::WAITING_DELAY;
            stopRobot();
            ROS_INFO_THROTTLE(1.0, "state=%s strategy=%s waiting for startup delay: %.1fs",
                              behaviorStateName(), strategy_.c_str(), startup_delay_sec_);
            return;
        }

        double robot_x = 0.0;
        double robot_y = 0.0;
        double robot_yaw = 0.0;
        if (!updateWorldState(robot_x, robot_y, robot_yaw)) {
            stopRobot();
            return;
        }
        updateBallStatesFromVision(robot_x, robot_y, robot_yaw);

        recordPathSample(robot_x, robot_y, robot_yaw);

        if (enable_recovery_ && handlePointRecoveryIfActive()) {
            return;
        }

        if (pickupBallsInContactBand(robot_x, robot_y, robot_yaw)) {
            behavior_state_ = BehaviorState::PICKUP_SUCCESS;
            stopRobot();
            return;
        }

        std::string target_name;
        if (!selectTargetWithLock(robot_x, robot_y, robot_yaw, target_name)) {
            current_target_name_.clear();

            if (!anyAvailableTargets()) {
                behavior_state_ = BehaviorState::STOPPED_NO_TARGET;
                stopRobot();
                setControlMode(ControlMode::ROTATE_ONLY);
                resetSearchTracking();
                resetTraversalTracking();
                markRunFinishedIfNeeded();
                ROS_INFO_THROTTLE(2.0, "state=%s strategy=%s planning_source=%s all balls picked or no selectable targets left.",
                                  behaviorStateName(), strategy_.c_str(), planning_source_.c_str());
                return;
            }

            if (traversal_active_) {
                behavior_state_ = BehaviorState::TRAVERSING;
                if (!updateTraversalWaypoint(robot_x, robot_y)) {
                    behavior_state_ = BehaviorState::STOPPED_NO_TARGET;
                    stopRobot();
                    resetTraversalTracking();
                    markRunFinishedIfNeeded();
                    ROS_INFO_THROTTLE(2.0, "state=%s strategy=%s completed spiral traversal without finding a ball. Robot stopped.",
                                      behaviorStateName(), strategy_.c_str());
                    return;
                }

                publishTraversalControl(robot_x, robot_y, robot_yaw);
                ROS_INFO_THROTTLE(1.0, "state=%s strategy=%s waypoint=%zu/%zu spiral traversal active.",
                                  behaviorStateName(), strategy_.c_str(),
                                  traversal_waypoint_index_ + 1, traversal_waypoints_.size());
                return;
            }

            behavior_state_ = BehaviorState::SEARCHING;
            beginSearch(robot_yaw);
            if (isSearchBrakeActive()) {
                stopRobot();
                ROS_INFO_THROTTLE(1.0, "state=%s strategy=%s braking before in-place search rotation.",
                                  behaviorStateName(), strategy_.c_str());
                return;
            }
            if (updateSearchProgress(robot_yaw)) {
                resetSearchTracking();
                if (enable_spiral_traversal_) {
                    beginSpiralTraversal();
                    if (traversal_active_) {
                        behavior_state_ = BehaviorState::TRAVERSING;
                        publishTraversalControl(robot_x, robot_y, robot_yaw);
                        ROS_INFO_THROTTLE(1.0, "state=%s strategy=%s switching from search to spiral traversal.",
                                          behaviorStateName(), strategy_.c_str());
                        return;
                    }
                }

                behavior_state_ = BehaviorState::STOPPED_NO_TARGET;
                stopRobot();
                markRunFinishedIfNeeded();
                ROS_INFO_THROTTLE(2.0, "state=%s strategy=%s completed one full search rotation without finding a ball. Robot stopped.",
                                  behaviorStateName(), strategy_.c_str());
                return;
            }

            publishSearchRotation();
            ROS_INFO_THROTTLE(1.0, "state=%s strategy=%s search_yaw=%.3f rotating in place to search for balls.",
                              behaviorStateName(), strategy_.c_str(), search_accumulated_yaw_rad_);
            return;
        }
        resetSearchTracking();
        resetTraversalTracking();
        behavior_state_ = BehaviorState::TRACKING;

        const BallInfo& target_ball = balls_.at(target_name);

        double rel_forward = 0.0;
        double rel_lateral = 0.0;
        double heading = 0.0;
        double distance = 0.0;
        computeRelativeTarget(target_ball, robot_x, robot_y, robot_yaw, rel_forward, rel_lateral, heading, distance);
        double model_rel_forward = 0.0;
        double model_rel_lateral = 0.0;
        double model_heading = 0.0;
        double model_distance = 0.0;
        const bool have_model_target = computeRelativeTargetFromModel(target_ball,
                                                                      robot_x,
                                                                      robot_y,
                                                                      robot_yaw,
                                                                      model_rel_forward,
                                                                      model_rel_lateral,
                                                                      model_heading,
                                                                      model_distance);
        const double progress_heading = computeStrategyControlHeading(isPointStrategy(), rel_forward, rel_lateral);

        if (enable_recovery_) {
            if (control_mode_ == ControlMode::ROTATE_ONLY) {
                updatePointProgress(target_name,
                                    computeStrategyLateralOverflow(isPointStrategy(), rel_lateral),
                                    progress_heading);
                if (!point_last_progress_time_.isZero() &&
                    (ros::Time::now() - point_last_progress_time_).toSec() >= point_stuck_timeout_sec_) {
                    beginPointRecovery(rel_lateral);
                    return;
                }
            } else {
                point_last_progress_time_ = ros::Time::now();
            }
        }

        if (have_model_target &&
            isPickupSuccess(isPointStrategy(), model_rel_forward, model_rel_lateral, model_heading)) {
            behavior_state_ = BehaviorState::PICKUP_SUCCESS;
            stopRobot();
            removePickedBall(target_name);
            return;
        }

        publishControl(rel_forward, rel_lateral, heading, distance);

        ROS_INFO_THROTTLE(1.0,
                          "state=%s target=%s forward=%.3f lateral=%.3f heading=%.3f control_heading=%.3f lateral_overflow=%.3f strategy=%s mode=%s",
                          behaviorStateName(), target_name.c_str(), rel_forward, rel_lateral, heading,
                          computeStrategyControlHeading(isPointStrategy(), rel_forward, rel_lateral),
                          computeStrategyLateralOverflow(isPointStrategy(), rel_lateral),
                          strategy_.c_str(), controlModeName());
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher cmd_pub_;
    ros::Publisher pickup_complete_pub_;
    ros::Subscriber model_states_sub_;
    ros::Subscriber stereo_ball_positions_sub_;
    ros::Timer control_timer_;
    ros::ServiceClient delete_model_client_;
    ros::ServiceServer start_service_;

    gazebo_msgs::ModelStates latest_states_;
    bool have_model_states_ = false;
    geometry_msgs::PoseArray latest_stereo_ball_positions_;
    bool have_stereo_ball_positions_ = false;

    std::string robot_model_name_;
    std::string strategy_;
    std::string planning_source_;
    std::vector<std::string> ball_names_;
    std::map<std::string, BallInfo> balls_;

    double control_frequency_ = 20.0;
    double max_linear_speed_ = 0.4;
    double max_angular_speed_ = 0.8;
    double heading_gain_ = 1.2;
    double distance_gain_ = 0.5;
    double front_boundary_x_ = 0.6917;
    double pickup_half_width_ = 0.25;
    double ball_radius_ = 0.033;
    double pickup_tolerance_ = 0.02;
    double x_error_margin_ = 0.0241;
    double depth_error_margin_ = 0.0356;
    double segment_pickup_side_shrink_ = 0.0356;
    double segment_guidance_inset_ = 0.0356;
    double shared_advance_lateral_overflow_tolerance_ = 0.04;
    double shared_advance_heading_tolerance_ = 0.14;
    double shared_realign_lateral_overflow_tolerance_ = 0.10;
    double shared_realign_heading_tolerance_ = 0.28;
    double shared_realign_forward_error_threshold_ = 0.20;
    double shared_advance_heading_deadband_ = 0.06;
    double shared_advance_lateral_deadband_ = 0.02;
    double shared_advance_min_linear_speed_ = 0.08;
    double shared_advance_min_angular_speed_ = 0.05;
    double shared_advance_max_angular_speed_ = 0.18;
    double shared_rotate_min_angular_speed_ = 0.10;
    bool enable_recovery_ = false;
    double point_pick_heading_tolerance_ = 0.10;
    double point_pick_lateral_tolerance_ = 0.05;
    double point_stuck_timeout_sec_ = 1.5;
    double point_progress_score_epsilon_ = 0.015;
    double point_recovery_forward_speed_ = 0.08;
    double point_recovery_turn_speed_ = 0.45;
    double point_recovery_duration_sec_ = 0.60;
    double point_heading_lookahead_ = 0.35;
    double point_close_advance_window_ = 0.10;
    double point_close_lateral_exit_tolerance_ = 0.12;
    double point_advance_max_angular_speed_ = 0.16;
    double advance_heading_deadband_ = 0.08;
    double point_advance_lateral_deadband_ = 0.05;
    double point_advance_enter_heading_tolerance_ = 0.22;
    double point_advance_enter_lateral_tolerance_ = 0.10;
    double point_advance_exit_heading_tolerance_ = 0.45;
    double point_advance_exit_lateral_tolerance_ = 0.18;
    double point_rotate_recover_forward_threshold_ = 0.45;
    double segment_advance_lateral_deadband_ = 0.02;
    double segment_rotate_min_angular_speed_ = 0.10;
    double point_realign_heading_tolerance_ = 0.26;
    double point_realign_lateral_tolerance_ = 0.16;
    double segment_realign_heading_tolerance_ = 0.30;
    double segment_realign_lateral_tolerance_ = 0.22;
    double control_mode_hold_sec_ = 0.25;
    int control_mode_switch_confirm_cycles_ = 4;
    double linear_cmd_slew_rate_ = 0.90;
    double angular_cmd_slew_rate_ = 1.80;
    double rotate_only_heading_threshold_ = 0.35;
    double forward_control_heading_threshold_ = 0.60;
    double target_loss_timeout_ = 0.5;
    double visual_target_match_distance_ = 0.60;
    double visual_positions_timeout_ = 0.50;
    double visual_position_ema_alpha_ = 0.35;
    double camera_offset_x_ = 0.25;
    double camera_offset_y_ = 0.0;
    double target_view_heading_half_angle_ = 0.75;
    double target_distance_tie_threshold_ = 0.10;
    double search_rotate_speed_ = 0.35;
    double search_brake_duration_sec_ = 0.25;
    bool enable_spiral_traversal_ = true;
    double traversal_x_min_ = 0.80;
    double traversal_x_max_ = 11.00;
    double traversal_y_min_ = -5.00;
    double traversal_y_max_ = 5.00;
    double traversal_inset_step_ = 1.20;
    double traversal_waypoint_tolerance_ = 0.25;
    double traversal_heading_tolerance_ = 0.22;
    double traversal_linear_speed_ = 0.22;
    double traversal_angular_gain_ = 1.20;
    double traversal_max_angular_speed_ = 0.45;
    double heading_deadband_ = 0.03;
    double lateral_deadband_ = 0.02;
    double segment_heading_enter_tolerance_ = 0.12;
    double segment_heading_exit_tolerance_ = 0.24;
    double segment_lateral_enter_tolerance_ = 0.10;
    double segment_lateral_exit_tolerance_ = 0.18;
    double segment_pickup_band_hysteresis_ = 0.06;
    double segment_advance_max_angular_speed_ = 0.22;
    double point_heading_exit_tolerance_ = 0.18;
    double point_lateral_exit_tolerance_ = 0.09;
    double startup_delay_sec_ = 0.0;
    bool wait_for_manual_start_ = false;
    bool manual_start_requested_ = true;
    std::string artifacts_output_dir_;
    std::vector<PathSample> path_samples_;
    double total_path_length_m_ = 0.0;
    double total_abs_yaw_change_rad_ = 0.0;
    std::size_t picked_ball_count_ = 0;
    bool run_started_ = false;
    bool run_finished_ = false;
    bool artifacts_written_ = false;
    ros::Time run_start_time_;
    ros::Time run_end_time_;
    bool point_progress_initialized_ = false;
    std::string point_progress_target_name_;
    double point_best_progress_score_ = std::numeric_limits<double>::max();
    ros::Time point_last_progress_time_;
    bool point_recovery_active_ = false;
    ros::Time point_recovery_end_time_;
    double point_recovery_turn_sign_ = 1.0;
    int point_recovery_count_ = 0;

    std::string current_target_name_;
    ControlMode control_mode_ = ControlMode::ROTATE_ONLY;
    BehaviorState behavior_state_ = BehaviorState::WAITING_START;
    ControlMode pending_control_mode_ = ControlMode::ROTATE_ONLY;
    bool pending_mode_switch_valid_ = false;
    int pending_mode_switch_count_ = 0;
    bool search_active_ = false;
    bool search_has_last_yaw_sample_ = false;
    double search_last_yaw_rad_ = 0.0;
    double search_accumulated_yaw_rad_ = 0.0;
    ros::Time search_brake_end_time_;
    bool traversal_active_ = false;
    std::vector<std::pair<double, double>> traversal_waypoints_;
    std::size_t traversal_waypoint_index_ = 0;
    ros::Time control_mode_last_switch_time_;
    ros::Time startup_reference_time_;
    bool startup_time_initialized_ = false;
    double last_linear_cmd_ = 0.0;
    double last_angular_cmd_ = 0.0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ball_pickup_controller");
    BallPickupController controller;
    ros::spin();
    return 0;
}
