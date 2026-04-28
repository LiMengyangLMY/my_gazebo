#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <utility>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

struct BoundingBox {
    int class_id = 0;
    std::string class_name;
    float confidence = 0.0f;
    cv::Rect rect;
    float cx = 0.0f;
    float cy = 0.0f;
};

struct StereoParameters {
    cv::Mat M1;
    cv::Mat d1;
    cv::Mat M2;
    cv::Mat d2;
    cv::Mat R;
    cv::Mat T;
};

struct LetterboxResult {
    cv::Mat image;
    float scale = 1.0f;
    int pad_x = 0;
    int pad_y = 0;
};

std::pair<double, double> compensateCoordinates(double z_fusion_cm, double x_est_cm) {
    double z_true = 0.0023 * std::pow(z_fusion_cm, 2) + 1.0168 * z_fusion_cm + 1.4368;
    z_true = std::max(15.0, z_true);

    double x_true = 1.4851 * x_est_cm - 0.0436 * z_fusion_cm + 6.6313;
    return {std::round(z_true * 10.0) / 10.0, std::round(x_true * 10.0) / 10.0};
}

bool ensureDirectory(const std::string& path) {
    if (path.empty()) {
        return false;
    }

    std::string current = (path.front() == '/') ? "/" : "";
    std::stringstream ss(path);
    std::string token;

    while (std::getline(ss, token, '/')) {
        if (token.empty()) {
            continue;
        }

        if (!current.empty() && current.back() != '/') {
            current += "/";
        }
        current += token;

        if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
            struct stat st {};
            if (::stat(current.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
                return false;
            }
        }
    }

    return true;
}

std::string fileNameOf(const std::string& path) {
    const std::size_t pos = path.find_last_of("/\\");
    return (pos == std::string::npos) ? path : path.substr(pos + 1);
}

std::string classNameForId(int class_id) {
    return (class_id == 0) ? "tennis_ball" : ("class_" + std::to_string(class_id));
}

StereoParameters loadStereoParameters(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("无法加载标定参数文件: " + path);
    }

    StereoParameters params;
    fs["M1"] >> params.M1;
    fs["d1"] >> params.d1;
    fs["M2"] >> params.M2;
    fs["d2"] >> params.d2;
    fs["R"] >> params.R;
    fs["T"] >> params.T;
    fs.release();

    if (params.M1.empty() || params.d1.empty() || params.M2.empty() || params.d2.empty() ||
        params.R.empty() || params.T.empty()) {
        throw std::runtime_error("标定参数文件缺少必要字段: " + path);
    }

    return params;
}

LetterboxResult letterboxImage(const cv::Mat& src, const cv::Size& target_size) {
    LetterboxResult result;
    const float scale = std::min(static_cast<float>(target_size.width) / static_cast<float>(src.cols),
                                 static_cast<float>(target_size.height) / static_cast<float>(src.rows));
    const int resized_w = static_cast<int>(std::round(src.cols * scale));
    const int resized_h = static_cast<int>(std::round(src.rows * scale));
    const int pad_x = (target_size.width - resized_w) / 2;
    const int pad_y = (target_size.height - resized_h) / 2;

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(resized_w, resized_h));

    cv::Mat canvas(target_size, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(canvas(cv::Rect(pad_x, pad_y, resized_w, resized_h)));

    result.image = canvas;
    result.scale = scale;
    result.pad_x = pad_x;
    result.pad_y = pad_y;
    return result;
}

cv::Mat reshapeDetections(const cv::Mat& output) {
    if (output.empty()) {
        throw std::runtime_error("网络输出为空。");
    }

    if (output.dims == 2) {
        cv::Mat detections = output;
        if (detections.type() != CV_32F) {
            detections.convertTo(detections, CV_32F);
        }
        return detections;
    }

    if (output.dims == 3) {
        const int dim1 = output.size[1];
        const int dim2 = output.size[2];
        cv::Mat detections;

        if (dim1 < dim2) {
            cv::Mat raw(dim1, dim2, CV_32F, const_cast<uchar*>(output.ptr()));
            cv::transpose(raw, detections);
        } else {
            detections = cv::Mat(dim1, dim2, CV_32F, const_cast<uchar*>(output.ptr())).clone();
        }
        return detections;
    }

    if (output.dims == 4 && output.size[0] == 1 && output.size[1] == 1) {
        return cv::Mat(output.size[2], output.size[3], CV_32F, const_cast<uchar*>(output.ptr())).clone();
    }

    throw std::runtime_error("暂不支持的 YOLO 输出维度。");
}

std::vector<BoundingBox> detectObjects(cv::Mat& img,
                                       cv::dnn::Net& net,
                                       float conf_threshold = 0.25f,
                                       float nms_threshold = 0.45f,
                                       const cv::Size& input_size = cv::Size(640, 640)) {
    std::vector<BoundingBox> boxes;

    LetterboxResult letterbox = letterboxImage(img, input_size);
    cv::Mat blob = cv::dnn::blobFromImage(letterbox.image, 1.0 / 255.0, input_size, cv::Scalar(), true, false);
    net.setInput(blob);

    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());
    if (outputs.empty()) {
        return boxes;
    }

    cv::Mat detections = reshapeDetections(outputs[0]);
    if (detections.cols < 5) {
        return boxes;
    }

    std::vector<cv::Rect> candidate_rects;
    std::vector<float> candidate_scores;
    std::vector<int> candidate_class_ids;

    for (int i = 0; i < detections.rows; ++i) {
        const float* data = detections.ptr<float>(i);
        cv::Mat scores = detections.row(i).colRange(4, detections.cols);

        double max_class_score = 0.0;
        cv::Point class_id_point;
        cv::minMaxLoc(scores, nullptr, &max_class_score, nullptr, &class_id_point);

        if (max_class_score < conf_threshold) {
            continue;
        }

        const float cx = data[0];
        const float cy = data[1];
        const float width = data[2];
        const float height = data[3];

        float x1 = (cx - width * 0.5f - static_cast<float>(letterbox.pad_x)) / letterbox.scale;
        float y1 = (cy - height * 0.5f - static_cast<float>(letterbox.pad_y)) / letterbox.scale;
        float x2 = (cx + width * 0.5f - static_cast<float>(letterbox.pad_x)) / letterbox.scale;
        float y2 = (cy + height * 0.5f - static_cast<float>(letterbox.pad_y)) / letterbox.scale;

        x1 = std::max(0.0f, std::min(x1, static_cast<float>(img.cols - 1)));
        y1 = std::max(0.0f, std::min(y1, static_cast<float>(img.rows - 1)));
        x2 = std::max(0.0f, std::min(x2, static_cast<float>(img.cols - 1)));
        y2 = std::max(0.0f, std::min(y2, static_cast<float>(img.rows - 1)));

        const int box_w = static_cast<int>(std::round(x2 - x1));
        const int box_h = static_cast<int>(std::round(y2 - y1));
        if (box_w <= 1 || box_h <= 1) {
            continue;
        }

        candidate_rects.emplace_back(static_cast<int>(std::round(x1)),
                                     static_cast<int>(std::round(y1)),
                                     box_w,
                                     box_h);
        candidate_scores.push_back(static_cast<float>(max_class_score));
        candidate_class_ids.push_back(class_id_point.x);
    }

    std::vector<int> nms_indices;
    cv::dnn::NMSBoxes(candidate_rects, candidate_scores, conf_threshold, nms_threshold, nms_indices);

    for (int idx : nms_indices) {
        const cv::Rect& rect = candidate_rects[idx];
        BoundingBox box;
        box.class_id = candidate_class_ids[idx];
        box.class_name = classNameForId(box.class_id);
        box.confidence = candidate_scores[idx];
        box.rect = rect;
        box.cx = rect.x + rect.width / 2.0f;
        box.cy = rect.y + rect.height / 2.0f;
        boxes.push_back(box);
    }

    return boxes;
}

std::vector<int> hungarianAssignment(const std::vector<std::vector<double>>& cost_matrix) {
    if (cost_matrix.empty() || cost_matrix[0].empty()) {
        return {};
    }

    const int rows = static_cast<int>(cost_matrix.size());
    const int cols = static_cast<int>(cost_matrix[0].size());
    const int dim = std::max(rows, cols);
    const double inf = 1e12;

    std::vector<std::vector<double>> cost(dim + 1, std::vector<double>(dim + 1, 1e6));
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            cost[i + 1][j + 1] = cost_matrix[i][j];
        }
    }

    std::vector<double> u(dim + 1, 0.0), v(dim + 1, 0.0), minv(dim + 1, 0.0);
    std::vector<int> p(dim + 1, 0), way(dim + 1, 0);

    for (int i = 1; i <= dim; ++i) {
        p[0] = i;
        int j0 = 0;
        std::fill(minv.begin(), minv.end(), inf);
        std::vector<char> used(dim + 1, false);

        do {
            used[j0] = true;
            const int i0 = p[j0];
            int j1 = 0;
            double delta = inf;

            for (int j = 1; j <= dim; ++j) {
                if (used[j]) {
                    continue;
                }

                const double cur = cost[i0][j] - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }

                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }

            for (int j = 0; j <= dim; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    std::vector<int> assignment(rows, -1);
    for (int j = 1; j <= dim; ++j) {
        if (p[j] >= 1 && p[j] <= rows && j <= cols) {
            assignment[p[j] - 1] = j - 1;
        }
    }
    return assignment;
}

std::vector<std::pair<BoundingBox, BoundingBox>> matchBoundingBoxes(std::vector<BoundingBox> boxes_l,
                                                                    std::vector<BoundingBox> boxes_r) {
    std::vector<std::pair<BoundingBox, BoundingBox>> matched_pairs;
    if (boxes_l.empty() || boxes_r.empty()) {
        return matched_pairs;
    }

    std::sort(boxes_l.begin(), boxes_l.end(), [](const BoundingBox& a, const BoundingBox& b) { return a.cx < b.cx; });
    std::sort(boxes_r.begin(), boxes_r.end(), [](const BoundingBox& a, const BoundingBox& b) { return a.cx < b.cx; });

    std::vector<std::vector<double>> cost_matrix(boxes_l.size(), std::vector<double>(boxes_r.size(), 1e6));
    for (std::size_t i = 0; i < boxes_l.size(); ++i) {
        for (std::size_t j = 0; j < boxes_r.size(); ++j) {
            const BoundingBox& bl = boxes_l[i];
            const BoundingBox& br = boxes_r[j];
            if (bl.class_id != br.class_id) {
                continue;
            }

            const double y_diff = std::abs(bl.cy - br.cy);
            const double x_diff = bl.cx - br.cx;
            const double area_l = std::max(1, bl.rect.area());
            const double area_r = std::max(1, br.rect.area());
            const double area_diff_ratio = std::abs(area_l - area_r) / std::max(area_l, area_r);

            if (y_diff > 50.0 || x_diff <= 0.0) {
                continue;
            }

            cost_matrix[i][j] = 1.0 * y_diff + 100.0 * area_diff_ratio;
        }
    }

    const std::vector<int> assignment = hungarianAssignment(cost_matrix);
    for (std::size_t i = 0; i < assignment.size(); ++i) {
        const int j = assignment[i];
        if (j >= 0 && cost_matrix[i][j] < 1e5) {
            matched_pairs.push_back({boxes_l[i], boxes_r[j]});
        }
    }
    return matched_pairs;
}

cv::Mat imageMsgToBgr(const sensor_msgs::ImageConstPtr& msg) {
    if (msg->encoding != "bgr8" && msg->encoding != "rgb8") {
        throw std::runtime_error("仅支持 bgr8/rgb8 图像编码，当前收到: " + msg->encoding);
    }

    const int type = CV_8UC3;
    cv::Mat wrapped(static_cast<int>(msg->height),
                    static_cast<int>(msg->width),
                    type,
                    const_cast<unsigned char*>(msg->data.data()),
                    static_cast<size_t>(msg->step));

    cv::Mat image = wrapped.clone();
    if (msg->encoding == "rgb8") {
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    }
    return image;
}

sensor_msgs::ImagePtr bgrToImageMsg(const cv::Mat& image,
                                    const std_msgs::Header& header,
                                    const std::string& encoding = "bgr8") {
    sensor_msgs::ImagePtr msg(new sensor_msgs::Image());
    msg->header = header;
    msg->height = static_cast<uint32_t>(image.rows);
    msg->width = static_cast<uint32_t>(image.cols);
    msg->encoding = encoding;
    msg->is_bigendian = false;
    msg->step = static_cast<sensor_msgs::Image::_step_type>(image.cols * image.elemSize());

    const std::size_t data_size = msg->step * image.rows;
    msg->data.resize(data_size);
    std::memcpy(msg->data.data(), image.data, data_size);
    return msg;
}

class StereoVisionNode {
public:
    StereoVisionNode()
        : nh_(),
          pnh_("~"),
          it_(nh_) {
        loadParameters();
        params_ = loadStereoParameters(config_path_);

        try {
            net_ = cv::dnn::readNetFromONNX(model_path_);
        } catch (const cv::Exception& e) {
            throw std::runtime_error(std::string("ONNX 模型加载失败: ") + e.what());
        }

        if (net_.empty()) {
            throw std::runtime_error("ONNX 模型加载失败: " + model_path_);
        }

        left_sub_ = it_.subscribe(left_topic_, 1, &StereoVisionNode::leftImageCallback, this);
        right_sub_ = it_.subscribe(right_topic_, 1, &StereoVisionNode::rightImageCallback, this);

        left_rect_pub_ = it_.advertise("stereo/left_rectified", 1);
        right_rect_pub_ = it_.advertise("stereo/right_rectified", 1);
        debug_pub_ = it_.advertise("stereo/debug_image", 1);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("stereo/ball_positions", 1);

        ROS_INFO("stereo_node started. left_topic=%s right_topic=%s model=%s config=%s",
                 left_topic_.c_str(), right_topic_.c_str(), model_path_.c_str(), config_path_.c_str());
    }

private:
    void loadParameters() {
        pnh_.param<std::string>("config_path", config_path_, "/home/lmy/catkin_ws/src/my_simulation/config/stereo_config.yaml");
        pnh_.param<std::string>("model_path", model_path_, "/home/lmy/catkin_ws/src/my_simulation/weights/best.onnx");
        pnh_.param<std::string>("left_topic", left_topic_, "/stereo_camera/left/image_raw");
        pnh_.param<std::string>("right_topic", right_topic_, "/stereo_camera/right/image_raw");
        pnh_.param<std::string>("output_frame", output_frame_, "camera_link");
        pnh_.param("sync_tolerance_sec", sync_tolerance_sec_, 0.10);
        pnh_.param("conf_threshold", conf_threshold_, 0.25f);
        pnh_.param("nms_threshold", nms_threshold_, 0.45f);
        pnh_.param("tennis_ball_diameter_mm", tennis_ball_diameter_mm_, 67.0);
    }

    void initializeRectificationIfNeeded(const cv::Size& img_size) {
        if (maps_ready_ && img_size == image_size_) {
            return;
        }

        image_size_ = img_size;
        cv::stereoRectify(params_.M1, params_.d1, params_.M2, params_.d2, image_size_,
                          params_.R, params_.T, R1_, R2_, P1_, P2_, Q_, cv::CALIB_ZERO_DISPARITY, 0);

        focal_length_ = P1_.at<double>(0, 0);
        cx_left_ = P1_.at<double>(0, 2);
        baseline_ = std::abs(params_.T.at<double>(0, 0));

        cv::initUndistortRectifyMap(params_.M1, params_.d1, R1_, P1_, image_size_, CV_32FC1, map1x_, map1y_);
        cv::initUndistortRectifyMap(params_.M2, params_.d2, R2_, P2_, image_size_, CV_32FC1, map2x_, map2y_);
        maps_ready_ = true;

        ROS_INFO("Rectification maps ready. image_size=%dx%d focal=%.3f baseline=%.3fmm",
                 image_size_.width, image_size_.height, focal_length_, baseline_);
    }

    void leftImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        latest_left_msg_ = msg;
        tryProcessPair();
    }

    void rightImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        latest_right_msg_ = msg;
        tryProcessPair();
    }

    void tryProcessPair() {
        if (!latest_left_msg_ || !latest_right_msg_) {
            return;
        }

        const double dt = std::abs((latest_left_msg_->header.stamp - latest_right_msg_->header.stamp).toSec());
        if (dt > sync_tolerance_sec_) {
            return;
        }

        sensor_msgs::ImageConstPtr left_msg = latest_left_msg_;
        sensor_msgs::ImageConstPtr right_msg = latest_right_msg_;
        latest_left_msg_.reset();
        latest_right_msg_.reset();
        processStereoPair(left_msg, right_msg);
    }

    void processStereoPair(const sensor_msgs::ImageConstPtr& left_msg,
                           const sensor_msgs::ImageConstPtr& right_msg) {
        cv::Mat left_image;
        cv::Mat right_image;

        try {
            left_image = imageMsgToBgr(left_msg);
            right_image = imageMsgToBgr(right_msg);
        } catch (const std::exception& e) {
            ROS_ERROR_THROTTLE(1.0, "image conversion failed: %s", e.what());
            return;
        }

        initializeRectificationIfNeeded(left_image.size());

        cv::Mat rectified_l, rectified_r;
        cv::remap(left_image, rectified_l, map1x_, map1y_, cv::INTER_LINEAR);
        cv::remap(right_image, rectified_r, map2x_, map2y_, cv::INTER_LINEAR);

        std::vector<BoundingBox> boxes_l = detectObjects(rectified_l, net_, conf_threshold_, nms_threshold_);
        std::vector<BoundingBox> boxes_r = detectObjects(rectified_r, net_, conf_threshold_, nms_threshold_);
        std::vector<std::pair<BoundingBox, BoundingBox>> matched_pairs = matchBoundingBoxes(boxes_l, boxes_r);

        cv::Mat combined_display;
        cv::hconcat(rectified_l, rectified_r, combined_display);
        const int img_width = rectified_l.cols;

        geometry_msgs::PoseArray pose_array;
        pose_array.header.stamp = left_msg->header.stamp;
        pose_array.header.frame_id = output_frame_;

        for (const auto& pair : matched_pairs) {
            const BoundingBox& bl = pair.first;
            const BoundingBox& br = pair.second;

            const double disparity = bl.cx - br.cx;
            if (disparity <= 0.0) {
                continue;
            }

            const double w_pixel = bl.rect.width;
            const double h_pixel = bl.rect.height;
            const double raw_depth_mm = (focal_length_ * baseline_) / disparity;
            const double size_pixel = (w_pixel + h_pixel) / 2.0;
            const double z_geom_mm = (size_pixel > 0.0)
                ? (focal_length_ * tennis_ball_diameter_mm_) / size_pixel
                : raw_depth_mm;

            const double z_fusion_mm = (raw_depth_mm < 1000.0)
                ? (0.5 * raw_depth_mm + 0.5 * z_geom_mm)
                : (0.3 * raw_depth_mm + 0.7 * z_geom_mm);

            const double raw_x_offset_left_cam_mm = (bl.cx - cx_left_) * z_fusion_mm / focal_length_;
            const double raw_x_offset_center_mm = raw_x_offset_left_cam_mm - (baseline_ / 2.0);

            const double z_fusion_cm = z_fusion_mm / 10.0;
            const double raw_x_offset_cm = raw_x_offset_center_mm / 10.0;
            const std::pair<double, double> compensated = compensateCoordinates(z_fusion_cm, raw_x_offset_cm);

            const std::string label = bl.class_name + " | Z:" +
                cv::format("%.1f", compensated.first) + "cm X:" +
                cv::format("%+.1f", compensated.second) + "cm";

            cv::rectangle(combined_display, bl.rect, cv::Scalar(0, 255, 0), 2);
            cv::putText(combined_display, label,
                        cv::Point(bl.rect.x, std::max(20, bl.rect.y - 10)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2);

            cv::Rect right_rect = br.rect;
            right_rect.x += img_width;
            cv::rectangle(combined_display, right_rect, cv::Scalar(255, 0, 0), 2);
            cv::line(combined_display,
                     cv::Point(static_cast<int>(bl.cx), static_cast<int>(bl.cy)),
                     cv::Point(static_cast<int>(br.cx) + img_width, static_cast<int>(br.cy)),
                     cv::Scalar(0, 255, 255), 2);

            geometry_msgs::Pose pose;
            pose.position.x = compensated.first / 100.0;
            pose.position.y = -compensated.second / 100.0;
            pose.position.z = 0.033;
            pose.orientation.w = 1.0;
            pose_array.poses.push_back(pose);
        }

        publishImage(left_rect_pub_, rectified_l, left_msg->header, "bgr8");
        publishImage(right_rect_pub_, rectified_r, right_msg->header, "bgr8");
        publishImage(debug_pub_, combined_display, left_msg->header, "bgr8");
        pose_pub_.publish(pose_array);

        ROS_INFO_THROTTLE(1.0, "stereo_node detections=%zu matches=%zu",
                          boxes_l.size(), matched_pairs.size());
    }

    void publishImage(image_transport::Publisher& publisher,
                      const cv::Mat& image,
                      const std_msgs::Header& header,
                      const std::string& encoding) {
        publisher.publish(bgrToImageMsg(image, header, encoding));
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber left_sub_;
    image_transport::Subscriber right_sub_;
    image_transport::Publisher left_rect_pub_;
    image_transport::Publisher right_rect_pub_;
    image_transport::Publisher debug_pub_;
    ros::Publisher pose_pub_;

    sensor_msgs::ImageConstPtr latest_left_msg_;
    sensor_msgs::ImageConstPtr latest_right_msg_;

    StereoParameters params_;
    cv::dnn::Net net_;

    std::string config_path_;
    std::string model_path_;
    std::string left_topic_;
    std::string right_topic_;
    std::string output_frame_;
    double sync_tolerance_sec_ = 0.10;
    float conf_threshold_ = 0.25f;
    float nms_threshold_ = 0.45f;
    double tennis_ball_diameter_mm_ = 67.0;

    bool maps_ready_ = false;
    cv::Size image_size_;
    cv::Mat R1_, R2_, P1_, P2_, Q_;
    cv::Mat map1x_, map1y_, map2x_, map2y_;
    double focal_length_ = 0.0;
    double cx_left_ = 0.0;
    double baseline_ = 0.0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "stereo_node");

    try {
        StereoVisionNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("stereo_node startup failed: %s", e.what());
        return -1;
    }

    return 0;
}
