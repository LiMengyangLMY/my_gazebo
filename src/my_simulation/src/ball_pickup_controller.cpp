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

#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

struct BallInfo {
    std::string name;
    double world_x = 0.0;
    double world_y = 0.0;
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
        model_states_sub_ = nh_.subscribe("/gazebo/model_states", 1, &BallPickupController::modelStatesCallback, this);
        start_service_ = pnh_.advertiseService("start", &BallPickupController::handleStartRequest, this);
        control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_),
                                         &BallPickupController::controlTimerCallback,
                                         this);

        for (const std::string& ball_name : ball_names_) {
            BallInfo info;
            info.name = ball_name;
            balls_[ball_name] = info;
        }

        ROS_INFO("ball_pickup_controller started. strategy=%s, robot_model=%s, balls=%zu, wait_for_manual_start=%s",
                 strategy_.c_str(), robot_model_name_.c_str(), ball_names_.size(),
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

    void loadParameters() {
        pnh_.param<std::string>("robot_model_name", robot_model_name_, "simple_car");
        pnh_.param<std::string>("strategy", strategy_, "segment");
        pnh_.param("control_frequency", control_frequency_, 20.0);
        pnh_.param("max_linear_speed", max_linear_speed_, 0.4);
        pnh_.param("max_angular_speed", max_angular_speed_, 0.8);
        pnh_.param("heading_gain", heading_gain_, 1.2);
        pnh_.param("distance_gain", distance_gain_, 0.5);
        pnh_.param("front_boundary_x", front_boundary_x_, 0.25);
        pnh_.param("pickup_half_width", pickup_half_width_, 0.25);
        pnh_.param("ball_radius", ball_radius_, 0.033);
        pnh_.param("pickup_tolerance", pickup_tolerance_, 0.02);
        pnh_.param("x_error_margin", x_error_margin_, 0.0241);
        pnh_.param("depth_error_margin", depth_error_margin_, 0.0356);
        pnh_.param("point_pick_heading_tolerance", point_pick_heading_tolerance_, 0.10);
        pnh_.param("point_pick_lateral_tolerance", point_pick_lateral_tolerance_, 0.05);
        pnh_.param("rotate_only_heading_threshold", rotate_only_heading_threshold_, 0.35);
        pnh_.param("forward_control_heading_threshold", forward_control_heading_threshold_, 0.60);
        pnh_.param("target_loss_timeout", target_loss_timeout_, 0.5);
        pnh_.param("heading_deadband", heading_deadband_, 0.03);
        pnh_.param("lateral_deadband", lateral_deadband_, 0.02);
        pnh_.param("segment_heading_enter_tolerance", segment_heading_enter_tolerance_, 0.12);
        pnh_.param("segment_heading_exit_tolerance", segment_heading_exit_tolerance_, 0.24);
        pnh_.param("segment_lateral_enter_tolerance", segment_lateral_enter_tolerance_, 0.10);
        pnh_.param("segment_lateral_exit_tolerance", segment_lateral_exit_tolerance_, 0.18);
        pnh_.param("point_heading_exit_tolerance", point_heading_exit_tolerance_, 0.18);
        pnh_.param("point_lateral_exit_tolerance", point_lateral_exit_tolerance_, 0.09);
        pnh_.param("startup_delay_sec", startup_delay_sec_, 0.0);
        pnh_.param("wait_for_manual_start", wait_for_manual_start_, false);
        pnh_.param<std::string>("artifacts_output_dir", artifacts_output_dir_, "");

        if (!pnh_.getParam("ball_names", ball_names_)) {
            ball_names_ = {"tennis_ball_1", "tennis_ball_2", "tennis_ball_3", "tennis_ball_4"};
        }

        manual_start_requested_ = !wait_for_manual_start_;
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        latest_states_ = *msg;
        have_model_states_ = true;
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
            entry.second.visible = false;
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
                it->second.world_x = latest_states_.pose[i].position.x;
                it->second.world_y = latest_states_.pose[i].position.y;
                it->second.visible = true;
                it->second.last_seen = ros::Time::now();
                if (!it->second.initial_recorded) {
                    it->second.initial_recorded = true;
                    it->second.initial_x = it->second.world_x;
                    it->second.initial_y = it->second.world_y;
                }
            }
        }

        return robot_found;
    }

    bool selectNearestTarget(double robot_x, double robot_y, std::string& target_name) {
        double best_distance = std::numeric_limits<double>::max();
        bool found = false;

        for (const auto& entry : balls_) {
            const BallInfo& ball = entry.second;
            if (ball.picked || !ball.visible) {
                continue;
            }

            const double dx = ball.world_x - robot_x;
            const double dy = ball.world_y - robot_y;
            const double distance = std::hypot(dx, dy);
            if (distance < best_distance) {
                best_distance = distance;
                target_name = ball.name;
                found = true;
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

        if (ball.visible) {
            return true;
        }

        return !ball.last_seen.isZero() && (ros::Time::now() - ball.last_seen).toSec() <= target_loss_timeout_;
    }

    bool selectTargetWithLock(double robot_x, double robot_y, std::string& target_name) {
        if (!current_target_name_.empty()) {
            auto current_it = balls_.find(current_target_name_);
            if (current_it != balls_.end() && ballAvailable(current_it->second)) {
                target_name = current_target_name_;
                return true;
            }
            current_target_name_.clear();
            control_mode_ = ControlMode::ROTATE_ONLY;
        }

        if (!selectNearestTarget(robot_x, robot_y, target_name)) {
            return false;
        }

        current_target_name_ = target_name;
        control_mode_ = ControlMode::ROTATE_ONLY;
        return true;
    }

    bool computeRelativeTarget(const BallInfo& ball,
                               double robot_x,
                               double robot_y,
                               double robot_yaw,
                               double& rel_forward,
                               double& rel_lateral,
                               double& heading,
                               double& distance) const {
        const double dx = ball.world_x - robot_x;
        const double dy = ball.world_y - robot_y;

        rel_forward = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
        rel_lateral = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
        heading = std::atan2(rel_lateral, rel_forward);
        distance = std::hypot(rel_forward, rel_lateral);
        return true;
    }

    bool isPickupSuccess(double rel_forward, double rel_lateral) const {
        const double effective_half_width = pickup_half_width_ + x_error_margin_;
        const double effective_depth_tolerance = ball_radius_ + pickup_tolerance_ + depth_error_margin_;

        return std::abs(rel_lateral) <= effective_half_width &&
               std::abs(rel_forward - front_boundary_x_) <= effective_depth_tolerance;
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
        if (current_target_name_ == ball_name) {
            current_target_name_.clear();
            control_mode_ = ControlMode::ROTATE_ONLY;
        }

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
        cmd_pub_.publish(cmd);
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
        control_mode_ = ControlMode::ROTATE_ONLY;

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
        const bool all_picked = (picked_ball_count_ == ball_names_.size());

        ofs << "# Pickup Experiment Summary\n\n";
        ofs << "- Strategy: `" << strategy_ << "`\n";
        ofs << "- Robot model: `" << robot_model_name_ << "`\n";
        ofs << "- Balls configured: `" << ball_names_.size() << "`\n";
        ofs << "- Balls picked: `" << picked_ball_count_ << "`\n";
        ofs << "- Run completed all pickups: `" << (all_picked ? "yes" : "no") << "`\n";
        ofs << "- Total pickup time (sim): `" << formatDouble(duration_sec, 3) << " s`\n";
        ofs << "- Total traveled distance: `" << formatDouble(total_path_length_m_, 3) << " m`\n";
        ofs << "- Average path speed: `" << formatDouble(avg_speed, 3) << " m/s`\n";
        ofs << "- Path samples: `" << path_samples_.size() << "`\n\n";

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

    void updateControlMode(bool point_strategy, double heading_abs, double lateral_abs) {
        if (point_strategy) {
            if (control_mode_ == ControlMode::ROTATE_ONLY) {
                if (heading_abs <= point_pick_heading_tolerance_ &&
                    lateral_abs <= point_pick_lateral_tolerance_) {
                    control_mode_ = ControlMode::ADVANCE;
                }
            } else {
                if (heading_abs > point_heading_exit_tolerance_ ||
                    lateral_abs > point_lateral_exit_tolerance_) {
                    control_mode_ = ControlMode::ROTATE_ONLY;
                }
            }
        } else {
            if (control_mode_ == ControlMode::ROTATE_ONLY) {
                if (heading_abs <= segment_heading_enter_tolerance_ &&
                    lateral_abs <= segment_lateral_enter_tolerance_) {
                    control_mode_ = ControlMode::ADVANCE;
                }
            } else {
                if (heading_abs > segment_heading_exit_tolerance_ ||
                    lateral_abs > segment_lateral_exit_tolerance_) {
                    control_mode_ = ControlMode::ROTATE_ONLY;
                }
            }
        }
    }

    void publishControl(double rel_forward, double rel_lateral, double heading, double distance) {
        geometry_msgs::Twist cmd;

        const bool point_strategy = (strategy_ == "point");
        const double heading_abs = std::abs(heading);
        const double lateral_abs = std::abs(rel_lateral);
        const double forward_error = std::max(0.0, rel_forward - front_boundary_x_);

        updateControlMode(point_strategy, heading_abs, lateral_abs);

        double heading_cmd = (heading_abs > heading_deadband_) ? heading : 0.0;
        double reduced_heading_gain = (forward_error < 0.50) ? (0.45 * heading_gain_) : (0.75 * heading_gain_);

        if (control_mode_ == ControlMode::ROTATE_ONLY) {
            cmd.linear.x = 0.0;
            cmd.angular.z = clamp(heading_gain_ * heading_cmd, -max_angular_speed_, max_angular_speed_);
        } else {
            double linear_cmd = distance_gain_ * forward_error;
            linear_cmd = clamp(linear_cmd, 0.0, max_linear_speed_);

            if (point_strategy) {
                linear_cmd = std::max(0.06, linear_cmd);
            } else {
                const double lateral_scale = (lateral_abs <= lateral_deadband_)
                    ? 1.0
                    : clamp(1.0 - lateral_abs / std::max(0.20, pickup_half_width_), 0.35, 1.0);
                linear_cmd *= lateral_scale;
                linear_cmd = std::max(0.08, linear_cmd);
            }

            if (forward_error < 0.10) {
                linear_cmd = std::min(linear_cmd, 0.10);
            } else if (forward_error < 0.20) {
                linear_cmd = std::min(linear_cmd, 0.18);
            }

            cmd.linear.x = linear_cmd;
            cmd.angular.z = clamp(reduced_heading_gain * heading_cmd, -max_angular_speed_, max_angular_speed_);
        }

        if (distance < front_boundary_x_) {
            cmd.linear.x = 0.0;
        }

        cmd_pub_.publish(cmd);
    }

    void controlTimerCallback(const ros::TimerEvent&) {
        if (wait_for_manual_start_ && !manual_start_requested_) {
            stopRobot();
            ROS_INFO_THROTTLE(1.0, "ball_pickup_controller waiting for manual start request.");
            return;
        }

        if (!startup_time_initialized_) {
            startup_reference_time_ = ros::Time::now();
            startup_time_initialized_ = true;
        }

        if ((ros::Time::now() - startup_reference_time_).toSec() < startup_delay_sec_) {
            stopRobot();
            ROS_INFO_THROTTLE(1.0, "ball_pickup_controller waiting for startup delay: %.1fs",
                              startup_delay_sec_);
            return;
        }

        double robot_x = 0.0;
        double robot_y = 0.0;
        double robot_yaw = 0.0;
        if (!updateWorldState(robot_x, robot_y, robot_yaw)) {
            stopRobot();
            return;
        }

        recordPathSample(robot_x, robot_y, robot_yaw);

        std::string target_name;
        if (!selectTargetWithLock(robot_x, robot_y, target_name)) {
            stopRobot();
            current_target_name_.clear();
            control_mode_ = ControlMode::ROTATE_ONLY;
            markRunFinishedIfNeeded();
            ROS_INFO_THROTTLE(2.0, "All balls picked or no visible targets left.");
            return;
        }

        const BallInfo& target_ball = balls_.at(target_name);

        double rel_forward = 0.0;
        double rel_lateral = 0.0;
        double heading = 0.0;
        double distance = 0.0;
        computeRelativeTarget(target_ball, robot_x, robot_y, robot_yaw, rel_forward, rel_lateral, heading, distance);

        if (isPickupSuccess(rel_forward, rel_lateral)) {
            stopRobot();
            removePickedBall(target_name);
            return;
        }

        publishControl(rel_forward, rel_lateral, heading, distance);

        ROS_INFO_THROTTLE(1.0,
                          "target=%s forward=%.3f lateral=%.3f heading=%.3f strategy=%s mode=%s",
                          target_name.c_str(), rel_forward, rel_lateral, heading, strategy_.c_str(), controlModeName());
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber model_states_sub_;
    ros::Timer control_timer_;
    ros::ServiceClient delete_model_client_;
    ros::ServiceServer start_service_;

    gazebo_msgs::ModelStates latest_states_;
    bool have_model_states_ = false;

    std::string robot_model_name_;
    std::string strategy_;
    std::vector<std::string> ball_names_;
    std::map<std::string, BallInfo> balls_;

    double control_frequency_ = 20.0;
    double max_linear_speed_ = 0.4;
    double max_angular_speed_ = 0.8;
    double heading_gain_ = 1.2;
    double distance_gain_ = 0.5;
    double front_boundary_x_ = 0.25;
    double pickup_half_width_ = 0.25;
    double ball_radius_ = 0.033;
    double pickup_tolerance_ = 0.02;
    double x_error_margin_ = 0.0241;
    double depth_error_margin_ = 0.0356;
    double point_pick_heading_tolerance_ = 0.10;
    double point_pick_lateral_tolerance_ = 0.05;
    double rotate_only_heading_threshold_ = 0.35;
    double forward_control_heading_threshold_ = 0.60;
    double target_loss_timeout_ = 0.5;
    double heading_deadband_ = 0.03;
    double lateral_deadband_ = 0.02;
    double segment_heading_enter_tolerance_ = 0.12;
    double segment_heading_exit_tolerance_ = 0.24;
    double segment_lateral_enter_tolerance_ = 0.10;
    double segment_lateral_exit_tolerance_ = 0.18;
    double point_heading_exit_tolerance_ = 0.18;
    double point_lateral_exit_tolerance_ = 0.09;
    double startup_delay_sec_ = 0.0;
    bool wait_for_manual_start_ = false;
    bool manual_start_requested_ = true;
    std::string artifacts_output_dir_;
    std::vector<PathSample> path_samples_;
    double total_path_length_m_ = 0.0;
    std::size_t picked_ball_count_ = 0;
    bool run_started_ = false;
    bool run_finished_ = false;
    bool artifacts_written_ = false;
    ros::Time run_start_time_;
    ros::Time run_end_time_;

    std::string current_target_name_;
    ControlMode control_mode_ = ControlMode::ROTATE_ONLY;
    ros::Time startup_reference_time_;
    bool startup_time_initialized_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ball_pickup_controller");
    BallPickupController controller;
    ros::spin();
    return 0;
}
