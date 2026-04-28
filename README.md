# 🎾 视觉定位模块: 网球识别定位及捡球路径规划

本模块是《基于计算机视觉的网球识别定位及捡球路径规划》的核心感知与仿真子系统。当前工程包含两部分：

- 纯 C++ 离线验证程序，基于 OpenCV DNN 运行 YOLO ONNX 模型，完成双目图像中的网球检测、左右目标匹配、融合深度估计、误差补偿和结果可视化
- Gazebo + ROS 实时仿真流程，接入双目相机话题、实时视觉节点与捡球控制节点，用于验证改进拾取策略和录制展示视频

## 功能概述

当前程序支持以下完整离线流程：

- 读取双目标定参数 `stereo_config.yaml`
- 加载 YOLO ONNX 模型 `best.onnx`
- 批量读取 `data_multi_test/left_*.jpg` 与 `right_*.jpg`
- 对左右图像执行立体校正
- 使用 OpenCV DNN 进行网球检测
- 使用匈牙利算法完成左右目标全局最优匹配
- 融合双目视差深度与基于网球物理尺寸的单目几何深度
- 使用经验补偿模型输出最终 `Z/X` 坐标
- 生成带检测框、匹配连线和坐标标签的结果图

## 核心算法

### 1. 深度融合策略

针对单一双目视差在不同距离下精度波动的问题，程序引入了网球已知直径 `67.0 mm` 的单目几何测距：

- 近场 `Z < 1m`：`Z_fusion = 0.5 * Z_raw_depth + 0.5 * Z_geom`
- 远场 `Z >= 1m`：`Z_fusion = 0.3 * Z_raw_depth + 0.7 * Z_geom`

### 2. 误差补偿模型

为减小系统误差，程序对融合深度和横向偏移进行经验补偿：

- `Z_true = 0.0023 * Z_fusion^2 + 1.0168 * Z_fusion + 1.4368`
- `X_true = 1.4851 * X_est - 0.0436 * Z_fusion + 6.6313`

输出坐标满足：

- `Z >= 15.0 cm`
- `Z/X` 保留 1 位小数

### 3. 左右目标匹配

程序先对左右检测框按横向坐标排序，再基于以下约束构建代价矩阵：

- 类别一致
- 极线约束：`|cy_left - cy_right| <= 50`
- 视差方向约束：`cx_left - cx_right > 0`
- 面积一致性约束

随后使用匈牙利算法求解全局最优匹配，而不是简单贪心匹配。

### 4. 捡球控制与路径规划创新点

后续项目中的捡球策略不再采用传统“车体中线对准球心后再接触”的点式拾取模型，而是采用“车前沿可触达区域”为目标的线段式拾取模型。

传统方案的控制目标通常是：

- 先旋转车体，使目标球尽量落到车体中线附近
- 再沿中线方向前进，直到与球接触
- 拾取目标本质上是车前方的一个点

本项目拟采用的方案是：

- 将车前沿看作一条有效拾取线段，而不是一个中心点
- 只要目标球进入这条前沿线段对应的可触达区域，就视为已经满足拾取条件
- 小车无需再额外旋转到“中线严格对准球心”，而是优先以最少转向完成接近和接触

从几何意义上看，这一改动等价于：

- 传统方案：捡球区域是一个点目标
- 本项目方案：捡球区域是一条线段目标

这意味着后续控制目标不再是单纯最小化球到车体中线的横向误差，而是最小化球到车前沿有效拾取线段的距离。只要球能够进入前沿捕获带，就可以完成拾取。

按当前项目设定，这条算法思想将结合以下规则实现：

- 前沿定义为 `base_link` 的前边界
- 有效拾取宽度按整车前宽计算
- 当球进入车前沿接触容差范围内，即判定为拾取成功
- 路径规划采用“最近可达球优先”，而不是“最近球且必须中线对准优先”
- 控制过程中允许边走边修正目标，不要求先完成静态对准再前进

这一策略的直接收益是：

- 减少不必要的原地转向和角度修正
- 缩短连续多球场景下的平均拾取时间
- 提高“就近拾取”策略的执行效率
- 使路径规划目标从“中心对准”转为“进入可拾取区域”

## 项目结构

```text
catkin_ws/
├── README.md
├── data_multi_test/
│   ├── left_0.jpg
│   ├── ...
│   └── right_3.jpg
└── src/
    └── my_simulation/
        ├── config/
        │   └── stereo_config.yaml
        ├── src/
        │   └── stereo_vision_localization.cpp
        └── weights/
            └── best.onnx
```

## 标定参数说明

当前程序从 `src/my_simulation/config/stereo_config.yaml` 读取以下字段：

- `M1`, `d1`: 左相机内参与畸变参数
- `M2`, `d2`: 右相机内参与畸变参数
- `R`, `T`: 双目外参

当前标定参数对应的主要物理量：

- 焦距约 `523.8 px`
- 基线长度约 `58.64 mm`

## 依赖要求

### 最低要求

- C++14
- OpenCV，需包含以下模块：
  - `core`
  - `imgproc`
  - `imgcodecs`
  - `highgui`
  - `calib3d`
  - `dnn`

### 版本建议

程序中使用的 `best.onnx` 在系统自带 OpenCV `4.2.0` 下无法正常加载，实际验证通过的版本为本地安装的 OpenCV `4.10.0`。

如果你在 OpenCV `4.2.0` 下运行，通常会出现类似错误：

```text
OpenCV(...)/onnx_importer.cpp:109: error: (-215:Assertion failed) ... in function 'getMatFromTensor'
```

因此建议：

- 不要用系统自带的 OpenCV `4.2.0` 直接运行该模型
- 使用独立安装的 OpenCV `4.10.x` 重新编译测试程序

## 编译方式

由于当前 ROS 工作区仍保留系统 OpenCV 依赖，离线测试建议先单独使用 `g++` 编译，而不是直接走 `catkin_make`。

假设新版 OpenCV 安装在 `/usr/local/opencv-4.10`：

```bash
cd /home/lmy/catkin_ws

g++ -std=c++14 src/my_simulation/src/stereo_vision_localization.cpp \
  -o /tmp/stereo_test \
  -I/usr/local/opencv-4.10/include/opencv4 \
  -L/usr/local/opencv-4.10/lib \
  -Wl,-rpath,/usr/local/opencv-4.10/lib \
  -lopencv_core \
  -lopencv_imgproc \
  -lopencv_imgcodecs \
  -lopencv_highgui \
  -lopencv_calib3d \
  -lopencv_dnn
```

## 运行方式

程序支持命令行参数，格式如下：

```bash
/tmp/stereo_test <config_path> <model_path> <data_dir> <result_dir>
```

例如：

```bash
/tmp/stereo_test \
  ./src/my_simulation/config/stereo_config.yaml \
  ./src/my_simulation/weights/best.onnx \
  ./data_multi_test \
  /tmp/result_multi_ball_final
```

如果不传参数，程序会使用源码中定义的默认路径。

## 输出结果

### 终端输出

程序会按图片逐行输出：

- 图片名
- 检测类别
- 原始视差深度 `Raw Z`
- 补偿后的最终深度 `Z`
- 补偿后的横向偏移 `X`

示例：

```text
| left_0.jpg | tennis_ball | 37.9 | 43.9 | 偏左 25.8 |
```

### 图像输出

结果图会保存到你指定的 `result_dir`，每张图包含：

- 左图绿色检测框
- 右图蓝色检测框
- 左右匹配目标之间的黄色连线
- 左图上的 `Z/X` 坐标标签
- 未成功匹配目标的红框和 `Unmatched` 标签

## 当前实现说明

当前版本是离线批处理程序，不是完整 ROS 话题节点。

当前入口函数做的是：

- 从磁盘读取测试图像
- 批量处理整个测试集
- 将结果保存到指定目录

还没有做的部分包括：

- 订阅 ROS 图像话题
- 实时接收单 USB 双目拼接画面
- 发布检测结果或三维坐标消息

## 后续 ROS 集成建议

如果后续要集成到 ROS，建议按下面的方向改造：

- 用 `cv_bridge` 替换当前 `cv::imread` 批处理入口
- 如果输入是单 USB 双目拼接图像，先按中线切分左右图
- 将离线批量处理改成单帧回调处理
- 视需要发布目标坐标、调试图像和检测框信息

示例切分逻辑：

```cpp
cv::Mat raw_img;
int half_width = raw_img.cols / 2;
cv::Mat img_l = raw_img(cv::Rect(0, 0, half_width, raw_img.rows));
cv::Mat img_r = raw_img(cv::Rect(half_width, 0, half_width, raw_img.rows));
```

## 当前已验证状态

截至当前版本，离线流程已经验证通过：

- 标定文件可以正常加载
- ONNX 模型可在 OpenCV `4.10.0` 下正常加载
- `data_multi_test` 中 4 组左右图可完整跑通
- 程序可输出检测结果、融合坐标和可视化图像

需要继续验证的部分：

- 不同距离下的真实测距误差
- 左右匹配是否在复杂遮挡场景下稳定
- 补偿模型在新数据集上的泛化效果
