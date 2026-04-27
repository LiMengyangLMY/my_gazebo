#include <algorithm>
#include <cerrno>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <utility>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

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

void printTableHeader() {
    std::cout << "\n" << std::string(95, '=') << "\n";
    std::cout << "| " << std::left << std::setw(13) << "图片名称"
              << " | " << std::setw(12) << "检测目标"
              << " | " << std::setw(14) << "视差深度(cm)"
              << " | " << std::setw(16) << "最终融合深度(cm)"
              << " | " << std::setw(15) << "最终X轴偏移(cm)"
              << " |\n";
    std::cout << std::string(95, '-') << "\n";
}

int main(int argc, char** argv) {
    const std::string config_path = (argc > 1) ? argv[1] : "./src/my_simulation/config/stereo_config.yaml";
    const std::string model_path = (argc > 2) ? argv[2] : "./src/my_simulation/weights/best.onnx";
    const std::string data_dir = (argc > 3) ? argv[3] : "./data_multi_test";
    const std::string result_dir = (argc > 4) ? argv[4] : "./result_multi_ball_final";
    const double tennis_ball_diameter_mm = 67.0;

    try {
        if (!ensureDirectory(result_dir)) {
            std::cerr << "❌ 无法创建结果目录: " << result_dir << std::endl;
            return -1;
        }
        std::cout << "📁 终极融合版结果存放目录已就绪: " << result_dir << std::endl;

        std::cout << "⏳ 正在加载双目参数和 YOLO ONNX 模型..." << std::endl;
        StereoParameters params = loadStereoParameters(config_path);

        cv::dnn::Net net;
        try {
            net = cv::dnn::readNetFromONNX(model_path);
        } catch (const cv::Exception& e) {
            std::cerr << "❌ ONNX 模型加载失败: " << e.what() << std::endl;
            std::cerr << "当前环境 OpenCV " << CV_VERSION
                      << " 可能无法解析该 ONNX；若仍失败，优先尝试升级 OpenCV 或重新导出较低 opset 的 ONNX。"
                      << std::endl;
            return -1;
        }

        if (net.empty()) {
            std::cerr << "❌ ONNX 模型加载失败: " << model_path << std::endl;
            return -1;
        }

        std::vector<cv::String> left_images;
        std::vector<cv::String> right_images;
        cv::glob(data_dir + "/left_*.jpg", left_images, false);
        cv::glob(data_dir + "/right_*.jpg", right_images, false);
        std::sort(left_images.begin(), left_images.end());
        std::sort(right_images.begin(), right_images.end());

        if (left_images.empty() || right_images.empty()) {
            std::cerr << "❌ 未在 " << data_dir << " 目录下找到测试图像！" << std::endl;
            return -1;
        }

        if (left_images.size() != right_images.size()) {
            std::cerr << "⚠️ 左右图数量不一致，将按最小数量进行处理。" << std::endl;
        }

        cv::Mat first_left = cv::imread(left_images.front());
        if (first_left.empty()) {
            std::cerr << "❌ 无法读取首张左图: " << left_images.front() << std::endl;
            return -1;
        }

        const cv::Size img_size = first_left.size();
        cv::Mat R1, R2, P1, P2, Q;
        cv::stereoRectify(params.M1, params.d1, params.M2, params.d2, img_size,
                          params.R, params.T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0);

        const double focal_length = P1.at<double>(0, 0);
        const double cx_left = P1.at<double>(0, 2);
        const double baseline = std::abs(params.T.at<double>(0, 0));

        cv::Mat map1x, map1y, map2x, map2y;
        cv::initUndistortRectifyMap(params.M1, params.d1, R1, P1, img_size, CV_32FC1, map1x, map1y);
        cv::initUndistortRectifyMap(params.M2, params.d2, R2, P2, img_size, CV_32FC1, map2x, map2y);

        printTableHeader();

        const std::size_t num_pairs = std::min(left_images.size(), right_images.size());
        for (std::size_t idx = 0; idx < num_pairs; ++idx) {
            const std::string left_path = left_images[idx];
            const std::string right_path = right_images[idx];
            const std::string filename = fileNameOf(left_path);

            cv::Mat img_l = cv::imread(left_path);
            cv::Mat img_r = cv::imread(right_path);
            if (img_l.empty() || img_r.empty()) {
                continue;
            }

            cv::Mat rectified_l, rectified_r;
            cv::remap(img_l, rectified_l, map1x, map1y, cv::INTER_LINEAR);
            cv::remap(img_r, rectified_r, map2x, map2y, cv::INTER_LINEAR);

            std::vector<BoundingBox> boxes_l = detectObjects(rectified_l, net);
            std::vector<BoundingBox> boxes_r = detectObjects(rectified_r, net);

            if (boxes_l.empty()) {
                std::cout << "| " << std::left << std::setw(13) << filename
                          << " | " << std::setw(16) << "未检测到目标"
                          << " | " << std::setw(14) << "-"
                          << " | " << std::setw(16) << "-"
                          << " | " << std::setw(15) << "-"
                          << " |\n";
            }

            cv::Mat combined_display;
            cv::hconcat(rectified_l, rectified_r, combined_display);
            const int img_width = rectified_l.cols;

            std::vector<std::pair<BoundingBox, BoundingBox>> matched_pairs = matchBoundingBoxes(boxes_l, boxes_r);

            for (const auto& pair : matched_pairs) {
                const BoundingBox& bl = pair.first;
                const BoundingBox& br = pair.second;

                const double disparity = bl.cx - br.cx;
                const double w_pixel = bl.rect.width;
                const double h_pixel = bl.rect.height;
                const double raw_depth_mm = (focal_length * baseline) / disparity;

                const double size_pixel = (w_pixel + h_pixel) / 2.0;
                const double z_geom_mm = (size_pixel > 0.0)
                    ? (focal_length * tennis_ball_diameter_mm) / size_pixel
                    : raw_depth_mm;

                const double z_fusion_mm = (raw_depth_mm < 1000.0)
                    ? (0.5 * raw_depth_mm + 0.5 * z_geom_mm)
                    : (0.3 * raw_depth_mm + 0.7 * z_geom_mm);

                const double raw_x_offset_left_cam_mm = (bl.cx - cx_left) * z_fusion_mm / focal_length;
                const double raw_x_offset_center_mm = raw_x_offset_left_cam_mm - (baseline / 2.0);

                const double z_fusion_cm = z_fusion_mm / 10.0;
                const double raw_x_offset_cm = raw_x_offset_center_mm / 10.0;
                const std::pair<double, double> compensated = compensateCoordinates(z_fusion_cm, raw_x_offset_cm);

                const std::string direction = (compensated.second < 0.0) ? "左" : "右";
                std::cout << "| " << std::left << std::setw(13) << filename
                          << " | " << std::setw(16) << bl.class_name
                          << " | " << std::setw(14) << std::fixed << std::setprecision(1) << (raw_depth_mm / 10.0)
                          << " | " << std::setw(16) << compensated.first
                          << " | 偏" << direction << " " << std::setw(12) << std::abs(compensated.second)
                          << " |\n";

                const std::string label = bl.class_name + " | Z:" +
                    cv::format("%.1f", compensated.first) + "cm X:" +
                    cv::format("%+.1f", compensated.second) + "cm";

                cv::rectangle(combined_display, bl.rect, cv::Scalar(0, 255, 0), 2);
                cv::putText(combined_display, label,
                            cv::Point(bl.rect.x, std::max(20, bl.rect.y - 10)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

                cv::Rect right_rect = br.rect;
                right_rect.x += img_width;
                cv::rectangle(combined_display, right_rect, cv::Scalar(255, 0, 0), 2);

                cv::line(combined_display,
                         cv::Point(static_cast<int>(bl.cx), static_cast<int>(bl.cy)),
                         cv::Point(static_cast<int>(br.cx) + img_width, static_cast<int>(br.cy)),
                         cv::Scalar(0, 255, 255), 2);
            }

            std::vector<cv::Rect> matched_left_rects;
            matched_left_rects.reserve(matched_pairs.size());
            for (const auto& pair : matched_pairs) {
                matched_left_rects.push_back(pair.first.rect);
            }

            for (const auto& bl : boxes_l) {
                const bool is_matched = std::find(matched_left_rects.begin(),
                                                  matched_left_rects.end(),
                                                  bl.rect) != matched_left_rects.end();
                if (is_matched) {
                    continue;
                }

                std::cout << "| " << std::left << std::setw(13) << filename
                          << " | " << std::setw(16) << bl.class_name
                          << " | " << std::setw(14) << "匹配失败"
                          << " | " << std::setw(16) << "-"
                          << " | " << std::setw(15) << "-"
                          << " |\n";

                cv::rectangle(combined_display, bl.rect, cv::Scalar(0, 0, 255), 2);
                cv::putText(combined_display, "Unmatched",
                            cv::Point(bl.rect.x, std::max(20, bl.rect.y - 10)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
            }

            const std::string save_path = result_dir + "/res_" + filename;
            cv::imwrite(save_path, combined_display);
        }

        std::cout << std::string(95, '=') << "\n";
        std::cout << "✅ 处理完毕！所有包含【完整连线】与【终极融合坐标】的图已保存至 "
                  << result_dir << "。" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "❌ 程序运行失败: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
