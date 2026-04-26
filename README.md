
# 🎾 视觉定位模块: 网球识别定位及捡球路径规划 (Tennis Vision & Localization)

本模块为《基于计算机视觉的网球识别定位及捡球路径规划》的核心感知子系统。系统采用纯 C++ 编写，基于 OpenCV DNN 部署 YOLO 轻量级目标检测模型，并自主实现了针对网球特征的多源深度融合与系统误差补偿算法，确保机械臂在执行捡球路径规划时获得高精度的三维物理坐标。

## 🌟 核心算法与特性

### 1. 深度数据融合策略 (Multi-source Depth Fusion)
针对单一双目视差测距在不同距离下精度波动的缺陷，系统引入了基于网球已知物理尺寸（直径 $67.0\text{ mm}$）的单目几何测距。系统根据目标距离动态调整置信权重：
* **近场高置信度 (Z < 1m)**: $Z_{fusion} = 0.5 \times Z_{raw\_depth} + 0.5 \times Z_{geom}$
* **远场几何主导 (Z $\ge$ 1m)**: $Z_{fusion} = 0.3 \times Z_{raw\_depth} + 0.7 \times Z_{geom}$

### 2. 多维误差补偿模型 (Error Compensation)
为消除相机镜头畸变残差及双目装配公差带来的系统误差，确保下位机控制精度，算法内嵌了校准补偿模型：
* **Z轴 (深度) 二次拟合补偿**: 
    $$Z_{true} = 0.0023 Z_{fusion}^2 + 1.0168 Z_{fusion} + 1.4368$$
* **X轴 (横向) 解耦多元线性回归**: 
    $$X_{true} = 1.4851 X_{est} - 0.0436 Z_{fusion} + 6.6313$$
*(注：输出坐标已限制机械臂运动学安全下限 $Z \ge 15.0\text{ cm}$，并进行小数点后一位截断处理)*

### 3. 稳健的立体匹配约束
抛弃算力消耗巨大的全局立体匹配算法，采用针对 YOLO 目标框的贪心匹配策略。结合极线几何约束（Y 轴像素差 $\le 50.0$）与视差方向约束（$X_{diff} > 0$），构建包含坐标与像素面积比的综合代价函数，实现极低延迟的目标配对。

## 📂 文件结构与配置说明


my_simulation/
├── config/
│   └── stereo_config.yaml         # 双目标定参数库
├── src/
│   └── stereo_vision_localization.cpp # 视觉融合测距核心算法源码
├── weights/
│   └── best.onnx                  # 训练完成的 YOLO ONNX 权重
└── data_multi_test/               # 离线验证数据集 (左右视图)
### 详细结构见文件 filetree.txt

### ⚙️ 硬件参数配置 (`stereo_config.yaml`)
当前模块加载的标准相机参数如下（由张正友标定法获取）：
* **焦距 (Focal Length)**: 约 $523.8\text{ px}$
* **基线长度 (Baseline)**: $58.63\text{ mm}$ (由平移向量 $T_x$ 绝对值提取)
* **畸变模型 (Distortion)**: 5 参数径向与切向畸变模型 ($k_1, k_2, p_1, p_2, k_3$)

## 🚀 离线基准测试运行

当前代码版本为纯 C++ 算法验证脚本，无需启动 ROS 节点即可独立验证测距算法的准确性。

**编译环境要求:**
* C++14 及以上标准
* OpenCV 3.4+ (需包含 `dnn` 与 `calib3d` 模块)

**执行流程:**
1. 确保 `stereo_config.yaml` 和 `best.onnx` 位于可执行文件同级或正确指定路径。
2. 将测试图像命名为 `left_img.jpg` 与 `right_img.jpg` 放入工作目录。
3. 编译并运行可执行文件，终端将输出目标的原始深度、补偿后 Z 坐标及 X 坐标。
```

***

### 💡 针对后续 ROS 集成的代码修改建议

当你准备将这段纯 C++ 算法接入单 USB 双目相机并转化为真正的 ROS 节点时，你需要在 `main()` 函数中替换掉 `cv::imread` 逻辑。你可以参考以下代码片段来分割单 USB 传输的宽幅画面：

```cpp
// 假设你通过 cv_bridge 拿到了单 USB 传回的原始宽幅图像 raw_img
// 例如 raw_img 的尺寸是 1280 x 480
cv::Mat raw_img; 

// 计算中线
int half_width = raw_img.cols / 2;

// 使用 cv::Rect 裁剪左右画面 (无需深拷贝，速度极快)
cv::Mat img_l = raw_img(cv::Rect(0, 0, half_width, raw_img.rows));
cv::Mat img_r = raw_img(cv::Rect(half_width, 0, half_width, raw_img.rows));

// 接下来就可以将 img_l 和 img_r 传入你写好的 cv::remap 和检测循环中了
```