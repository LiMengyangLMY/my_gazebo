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
        ├── models_and_worlds/
        │   ├── simple_car.urdf
        │   ├── spawn_car.launch
        │   └── tennis_court.world
        ├── scripts/
        │   └── run_sim_with_views_and_screen_record.sh
        ├── src/
        │   ├── stereo_vision_localization.cpp
        │   └── ball_pickup_controller.cpp
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

当前工程已经包含两条可直接运行的链路：

### 1. 离线双目识别定位验证

用于验证 `stereo_vision_localization.cpp` 的检测、匹配、深度融合与补偿逻辑：

- 从磁盘读取左右测试图像
- 批量处理整个测试集
- 保存带检测框、连线和坐标标签的结果图

### 2. Gazebo + ROS 实时仿真演示

用于完成网球拾取演示、录制展示视频和导出实验结果：

- Gazebo 载入网球场、小车、双目相机和网球模型
- `stereo_node` 实时订阅 `/stereo_camera/left/image_raw` 与 `/stereo_camera/right/image_raw`
- `ball_pickup_controller` 负责最近可达球优先的拾取控制
- 左右视图使用 `image_view` 打开
- 支持交互式整屏录制
- 一次实验结束后自动导出视频、路径图和指标文档

需要注意：

- 当前控制闭环主要基于 Gazebo 中的模型真值，视觉节点用于实时展示识别与定位结果
- 这样做的目的是保证当前阶段的演示稳定性，便于对比传统策略与改进策略的拾球时间

## ROS 仿真运行

如果只想直接启动仿真：

```bash
cd /home/lmy/catkin_ws
source /opt/ros/noetic/setup.bash
source /home/lmy/catkin_ws/devel/setup.bash
roslaunch my_simulation spawn_car.launch
```

如果只想手动查看左右相机视图：

```bash
rosrun image_view image_view image:=/stereo_camera/left/image_raw _autosize:=true _window_name:=left_camera_window
rosrun image_view image_view image:=/stereo_camera/right/image_raw _autosize:=true _window_name:=right_camera_window
```

## 交互式演示与录屏

推荐使用下面的脚本完成演示：

```bash
cd /home/lmy/catkin_ws
./src/my_simulation/scripts/run_sim_with_views_and_screen_record.sh
```

也可以自定义本次实验输出目录名：

```bash
cd /home/lmy/catkin_ws
./src/my_simulation/scripts/run_sim_with_views_and_screen_record.sh exp_case_1
```

脚本流程如下：

- 启动 Gazebo 仿真
- 打开左目和右目视图窗口
- 小车保持等待，不会提前开始捡球
- 你手动摆好 Gazebo 和左右视图窗口
- 按一次回车后，脚本先倒计时 `5s`
- 倒计时结束时同时开始整屏录制和小车捡球
- 按 `Ctrl+C` 结束实验并自动导出结果

## 实验结果输出

每次运行脚本后，都会在 `/mnt/hgfs/sharedFolder/` 下生成一个独立结果文件夹，例如：

```text
/mnt/hgfs/sharedFolder/exp_case_1/
```

该目录中包含：

- `screen.mp4`
  整个桌面的演示视频
- `path.csv`
  机器人平面路径采样数据
- `path.svg`
  机器人拾球平面路径图
- `metrics.md`
  拾球时间和实验指标汇总文档

### `metrics.md` 中包含的主要指标

- 总拾球时间
- 总行驶距离
- 平均路径速度
- 是否完成全部拾取
- 起点和终点位姿
- 每个球的初始位置
- 每个球的拾取顺序
- 每个球的拾取时间

## 当前已验证状态

截至当前版本，以下内容已经验证通过：

- 标定文件可以正常加载
- ONNX 模型可在 OpenCV `4.10.0` 下正常加载
- `data_multi_test` 中 4 组左右图可完整跑通
- Gazebo 双目相机话题可正常发布
- 左右视图可通过 `image_view` 正常显示
- 交互式脚本可完成倒计时后同时开始录屏和捡球
- 实验结束后可自动导出 `screen.mp4`、`path.csv`、`path.svg`、`metrics.md`

仍建议继续验证的部分：

- 不同距离下的真实测距误差
- 左右匹配在复杂遮挡场景下的稳定性
- 补偿模型在新数据集上的泛化效果
- 传统点目标策略与改进线段目标策略的多轮时间统计对比
