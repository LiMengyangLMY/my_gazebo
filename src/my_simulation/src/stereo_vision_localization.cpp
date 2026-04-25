#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

// ==========================================
// 数据结构定义
// ==========================================
struct BoundingBox {
    int class_id;
    std::string class_name;
    float confidence;
    cv::Rect rect;
    float cx;
    float cy;
};

// ==========================================
// 核心误差补偿模块 (基于多源特征融合模型)
// ==========================================
/**
 * @brief 对基于视差与几何融合计算的坐标进行系统误差补偿
 * @param z_fusion_cm 融合计算所得的深度坐标 (单位: cm)
 * @param x_est_cm 初始估计的横向坐标 (单位: cm)
 * @return std::pair<double, double> 补偿后的真实物理坐标 (Z, X)
 */
std::pair<double, double> compensateCoordinates(double z_fusion_cm, double x_est_cm) {
    // Z轴：二次多项式拟合校准模型
    double z_true = 0.0023 * std::pow(z_fusion_cm, 2) + 1.0168 * z_fusion_cm + 1.4368;
    z_true = std::max(15.0, z_true); // 设定机械臂运动学安全下限阈值
    
    // X轴：多元线性回归模型 (解耦偏航角与深度交叉影响)
    double x_true = 1.4851 * x_est_cm - 0.0436 * z_fusion_cm + 6.6313;
    
    // 保留一位小数以适配下位机控制精度
    z_true = std::round(z_true * 10.0) / 10.0;
    x_true = std::round(x_true * 10.0) / 10.0;
    
    return {z_true, x_true};
}

// ==========================================
// 目标检测与匹配模块
// ==========================================
/**
 * @brief 基于 OpenCV DNN 的目标检测函数 (需根据具体 ONNX 输出张量结构调整)
 */
std::vector<BoundingBox> detectObjects(cv::Mat& img, cv::dnn::Net& net) {
    std::vector<BoundingBox> boxes;
    // 此处省略 YOLO ONNX 的预处理(blobFromImage)与后处理(NMS)标准代码
    // 实际工程中需解析 net.forward() 的输出，提取置信度、类别与边界框
    return boxes;
}

/**
 * @brief 左右视图目标匹配 (基于代价矩阵的贪心匹配算法)
 */
std::vector<std::pair<BoundingBox, BoundingBox>> matchBoundingBoxes(
    std::vector<BoundingBox>& boxes_l, std::vector<BoundingBox>& boxes_r) {
    
    std::vector<std::pair<BoundingBox, BoundingBox>> matched_pairs;
    if (boxes_l.empty() || boxes_r.empty()) return matched_pairs;

    // 按照横向坐标排序以优化匹配效率
    std::sort(boxes_l.begin(), boxes_l.end(), [](const BoundingBox& a, const BoundingBox& b) { return a.cx < b.cx; });
    std::sort(boxes_r.begin(), boxes_r.end(), [](const BoundingBox& a, const BoundingBox& b) { return a.cx < b.cx; });

    for (const auto& bl : boxes_l) {
        double min_cost = 1e6;
        int best_match_idx = -1;

        for (size_t j = 0; j < boxes_r.size(); ++j) {
            const auto& br = boxes_r[j];
            if (bl.class_id != br.class_id) continue;

            double y_diff = std::abs(bl.cy - br.cy);
            double x_diff = bl.cx - br.cx; // 视差理论上应大于0
            
            double area_l = bl.rect.area();
            double area_r = br.rect.area();
            double area_diff_ratio = std::abs(area_l - area_r) / std::max(area_l, area_r);

            // 极线约束与视差方向约束
            if (y_diff > 50.0 || x_diff <= 0.0) continue;

            // 综合代价函数计算
            double cost = 1.0 * y_diff + 100.0 * area_diff_ratio;
            if (cost < min_cost) {
                min_cost = cost;
                best_match_idx = j;
            }
        }

        if (best_match_idx != -1 && min_cost < 1e5) {
            matched_pairs.push_back({bl, boxes_r[best_match_idx]});
            // 贪心策略下，为防止重复匹配，可将已匹配对象移出或标记
            // 此处为简化逻辑，实际项目中可引入第三方 Hungarian 算法库
        }
    }
    return matched_pairs;
}

// ==========================================
// 主控逻辑
// ==========================================
int main() {
    // 1. 初始化文件路径与物理常量
    const std::string param_path = "./stereo_config.yaml"; // 需提前转换
    const std::string model_path = "./best.onnx";
    const double TENNIS_BALL_DIAMETER_MM = 67.0;
    
    // 2. 加载相机标定矩阵参数
    cv::FileStorage fs(param_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Error: 无法加载标定参数文件。" << std::endl;
        return -1;
    }
    
    cv::Mat M1, d1, M2, d2, R, T;
    cv::Size img_size;
    fs["M1"] >> M1; fs["d1"] >> d1;
    fs["M2"] >> M2; fs["d2"] >> d2;
    fs["R"] >> R; fs["T"] >> T;
    // 此处省略读取 img_size 的代码，假设已知分辨率
    img_size = cv::Size(640, 480); 
    fs.release();

    // 3. 实例化深度学习模型
    cv::dnn::Net net = cv::dnn::readNetFromONNX(model_path);
    if (net.empty()) {
        std::cerr << "Error: 模型加载失败。" << std::endl;
        return -1;
    }

    // 4. 计算立体校正映射表
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(M1, d1, M2, d2, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0);

    double focal_length = P1.at<double>(0, 0);
    double cx_left = P1.at<double>(0, 2);
    double baseline = std::abs(T.at<double>(0, 0));

    cv::Mat map1x, map1y, map2x, map2y;
    cv::initUndistortRectifyMap(M1, d1, R1, P1, img_size, CV_32FC1, map1x, map1y);
    cv::initUndistortRectifyMap(M2, d2, R2, P2, img_size, CV_32FC1, map2x, map2y);

    // 5. 图像处理与测距推算主循环 (仿真环境中通常替换为 ROS Callback)
    cv::Mat img_l = cv::imread("./left_img.jpg");
    cv::Mat img_r = cv::imread("./right_img.jpg");
    if (img_l.empty() || img_r.empty()) return -1;

    cv::Mat rectified_l, rectified_r;
    cv::remap(img_l, rectified_l, map1x, map1y, cv::INTER_LINEAR);
    cv::remap(img_r, rectified_r, map2x, map2y, cv::INTER_LINEAR);

    std::vector<BoundingBox> boxes_l = detectObjects(rectified_l, net);
    std::vector<BoundingBox> boxes_r = detectObjects(rectified_r, net);

    auto matched_pairs = matchBoundingBoxes(boxes_l, boxes_r);

    for (const auto& pair : matched_pairs) {
        const BoundingBox& bl = pair.first;
        const BoundingBox& br = pair.second;

        // 特征参数提取
        double disparity = bl.cx - br.cx;
        double w_pixel = bl.rect.width;
        double h_pixel = bl.rect.height;

        // 测距算法 1：双目视差测距
        double raw_depth_mm = (focal_length * baseline) / disparity;

        // 测距算法 2：单目几何约束测距
        double size_pixel = (w_pixel + h_pixel) / 2.0;
        double z_geom_mm = (size_pixel > 0) ? ((focal_length * TENNIS_BALL_DIAMETER_MM) / size_pixel) : raw_depth_mm;

        // 深度数据融合策略
        double z_fusion_mm = (raw_depth_mm < 1000.0) ? 
                             (0.5 * raw_depth_mm + 0.5 * z_geom_mm) : 
                             (0.3 * raw_depth_mm + 0.7 * z_geom_mm);

        // 横向偏移量解算
        double raw_x_offset_left_cam_mm = (bl.cx - cx_left) * z_fusion_mm / focal_length;
        double raw_x_offset_center_mm = raw_x_offset_left_cam_mm - (baseline / 2.0);

        // 单位转换与误差补偿
        double z_fusion_cm = z_fusion_mm / 10.0;
        double raw_x_offset_cm = raw_x_offset_center_mm / 10.0;
        
        std::pair<double, double> compensated_coords = compensateCoordinates(z_fusion_cm, raw_x_offset_cm);
        
        std::cout << "Target: " << bl.class_name 
                  << " | Raw Z(cm): " << raw_depth_mm / 10.0 
                  << " | Compensated Z(cm): " << compensated_coords.first 
                  << " | Compensated X(cm): " << compensated_coords.second << std::endl;
    }

    return 0;
}