# 3.1 前言

本实验围绕“基于双目视觉的网球识别定位及自动拾捡”展开，目标是在 Gazebo + ROS 仿真环境中，对传统拾捡策略与改进拾捡策略进行可重复、可量化的对比分析。两种策略的核心差异不在于底层速度控制器，而在于“如何定义目标球已经被有效接近”的几何准则。传统策略将球视为一个需要尽量对准车体中线的点目标；改进策略则将球视为需要进入车前沿有效拾取带的区域目标，从而减少不必要的原地转向，提高连续多球场景下的拾取效率。

当前工程包含两条相互配合的链路。第一条是离线双目识别定位链路，负责完成左右图像校正、YOLO 网球检测、左右目标匹配、深度融合与误差补偿；第二条是 Gazebo + ROS 实时仿真链路，负责在固定场景中生成网球、驱动小车执行拾捡、记录路径与转向数据，并导出视频和指标文件。需要说明的是，当前实时仿真中的控制闭环仍主要基于 Gazebo 中的模型真值，以保证算法对比时的可控性和稳定性；视觉节点主要承担“识别、定位、显示和验证”的作用，而不是直接替代控制器的目标源。

从实验设计角度看，本研究关注的问题不是“小车能否捡到球”，而是“在同样的场景、同样的底层控制约束下，改进策略是否能减少小车转向次数、缩短拾取时间并降低路径摆动”。因此，实验重点落在以下几个方面：其一，小车结构参数和双目相机视场是否满足近场拾球要求；其二，传统与改进两类算法的目标几何定义差异如何影响控制行为；其三，如何通过合理的网球分布场景和量化指标，直观展示改进策略的性能优势。

# 3.2 实验环境

## 3.2.1 软件与仿真环境

实验运行环境由 ROS Noetic、Gazebo Classic、OpenCV DNN 与自定义控制节点构成。仿真入口文件为 [spawn_car.launch](/home/lmy/catkin_ws/src/my_simulation/models_and_worlds/spawn_car.launch:1)，其中完成以下工作：

- 加载网球场景 [tennis_court.world](/home/lmy/catkin_ws/src/my_simulation/models_and_worlds/tennis_court.world:1)
- 加载小车模型 [simple_car.urdf](/home/lmy/catkin_ws/src/my_simulation/models_and_worlds/simple_car.urdf:1)
- 启动双目视觉节点 [stereo_vision_localization.cpp](/home/lmy/catkin_ws/src/my_simulation/src/stereo_vision_localization.cpp:1)
- 启动拾捡控制节点 [ball_pickup_controller.cpp](/home/lmy/catkin_ws/src/my_simulation/src/ball_pickup_controller.cpp:1)
- 通过 [ball_scene_spawner.py](/home/lmy/catkin_ws/src/my_simulation/scripts/ball_scene_spawner.py:1) 动态生成场景中的网球

实验录制脚本为 [run_sim_with_views_and_screen_record.sh](/home/lmy/catkin_ws/src/my_simulation/scripts/run_sim_with_views_and_screen_record.sh:1)，脚本负责：

- 启动仿真与视觉窗口
- 触发实验开始
- 录制整屏视频
- 记录 `/cmd_vel` 与 `/odom` 数据
- 在最后一个球拾取完成后自动停止录制并导出结果

## 3.2.2 小车结构与尺寸

小车几何模型以 [simple_car.urdf](/home/lmy/catkin_ws/src/my_simulation/models_and_worlds/simple_car.urdf:1) 为准，俯视结构图见：

- [simple_car_three_view.svg](/home/lmy/catkin_ws/src/my_simulation/models_and_worlds/simple_car_three_view.svg:1)

图 3-1 给出了小车俯视结构示意图。小车主体采用近似方形底盘，前部安装双目相机，并在车头左右设置导流挡板，以便在近场将偏离中线的网球引导至车前拾取区域。为了便于后续实验分析，表 3-1 汇总了当前模型的主要几何参数。

**图 3-1 小车俯视结构示意图**

- 对应文件：[simple_car_three_view.svg](/home/lmy/catkin_ws/src/my_simulation/models_and_worlds/simple_car_three_view.svg:1)

**表 3-1 小车主要结构尺寸**

| 部件 | 参数含义 | 数值 |
| --- | --- | --- |
| 机身主体 | 长 × 宽 × 高 | `0.50 m × 0.50 m × 0.06 m` |
| 轮子 | 直径 × 厚度 | `0.06 m × 0.03 m` |
| 车轮布局 | 左右轮中心距 | `0.54 m` |
| 车轮布局 | 前后轮中心距 | `0.36 m` |
| 相机外壳 | 长 × 宽 × 高 | `0.02 m × 0.10 m × 0.02 m` |
| 相机安装点 | 相对 `base_link` | `(0.25, 0, 0.05) m` |
| 单侧挡板导流臂 | 长 × 宽 × 高 | `0.55 m × 0.018 m × 0.03 m` |
| 挡板末端圆柱 | 长度 × 半径 | `0.05 m × 0.04 m` |
| 挡板根部安装点 | 相对 `base_link` | `(0.40, ±0.24, -0.005) m` |
| 挡板安装偏航角 | 左/右 | `±0.60 rad` |
| 挡板末端圆柱中心 | 沿挡板局部前向偏移 | `0.305 m` |
| 整车外廓 | 长 × 宽 × 高 | `0.942 m × 0.904 m × 0.090 m` |

这两个前导流挡板的作用有二：一是将近场偏离中线的球引导到车体前沿区域，二是扩展“物理可拾取区”而不必完全依赖相机在极近距离继续稳定看到球。为了和结构保持一致，控制器中还引入了“格挡捕获区”判定，只要网球进入该区域，就可视为已经被揽入并完成拾取。

## 3.2.3 双目相机视野与配置

双目相机安装在车体前部，使用 Gazebo `multicamera` 传感器模拟。对应设置见 [simple_car.urdf](/home/lmy/catkin_ws/src/my_simulation/models_and_worlds/simple_car.urdf:171)。

当前主要参数如下：

**表 3-2 双目相机主要参数**

| 参数 | 数值 |
| --- | --- |
| 左右相机水平视场角 | `1.3962634 rad`（约 `80°`） |
| 图像分辨率 | `320 × 240` |
| 左右光学中心相对 `camera_link` 偏移 | `±0.03 m` |
| 等效基线 | `0.06 m` |
| 近裁剪面 | `0.02 m` |
| 远裁剪面 | `300 m` |

按当前俯视图分析，相机在格挡最外沿前向距离处的可见宽度约为 `0.674 m`。由于挡板外廓宽度大于该宽度，近场偏外侧网球可能在物理上已经进入挡板导流范围，但在图像中不再完整可见。因此，实验中采用“视觉引导 + 格挡捕获区判定”的组合方案，以减轻近场视野不足对拾取成功率的影响。

# 3.3 实验方法

## 3.3.1 双目识别与定位流程

从双目相机获取图像后，视觉模块按以下流程工作：

1. 读取左右相机图像，并根据 [stereo_config.yaml](/home/lmy/catkin_ws/src/my_simulation/config/stereo_config.yaml:1) 完成立体校正。
2. 对左右校正图分别调用 YOLO ONNX 模型进行网球检测，核心实现见 [detectObjects()](/home/lmy/catkin_ws/src/my_simulation/src/stereo_vision_localization.cpp:177)。
3. 基于类别一致、极线约束、视差方向约束和面积一致性约束，构建左右检测框的代价矩阵，并使用匈牙利算法完成全局最优匹配，匹配实现在 [matchBoundingBoxes()](/home/lmy/catkin_ws/src/my_simulation/src/stereo_vision_localization.cpp:338)。
4. 根据左右匹配框中心的视差估计双目深度，同时利用网球已知直径 `67 mm` 进行单目几何测距。
5. 将双目视差深度与单目几何深度按距离分段加权融合，再进行经验误差补偿，最终输出球在相机坐标系下的相对位置。
6. 将带检测框、匹配连线和坐标标签的图像发布到调试窗口，同时发布球的位置数组供上层验证使用。

该流程对应的核心代码在 [processStereoPair()](/home/lmy/catkin_ws/src/my_simulation/src/stereo_vision_localization.cpp:509)。

识别与定位流程可概括为：

```text
双目原始图像
  -> 立体校正
  -> 左右图 YOLO 检测
  -> 左右目标匹配（匈牙利算法）
  -> 视差深度 + 单目几何深度
  -> 深度融合与误差补偿
  -> 输出球的相对坐标
```

**图 3-2 双目识别与定位流程示意**

```text
左目图像 --------\
                  -> 立体校正 -> YOLO 检测 -> 左右目标匹配 -> 深度融合 -> 误差补偿 -> 网球相对坐标
右目图像 --------/
```

## 3.3.2 捡球控制策略

当前控制器的高层状态包含：

- `WAITING_START`
- `WAITING_DELAY`
- `SEARCHING`
- `TRACKING`
- `PICKUP_SUCCESS`
- `STOPPED_NO_TARGET`

当控制器进入 `TRACKING` 状态后，小车的低层模式只有两类：

- `ROTATE_ONLY`：原地调整角度
- `ADVANCE`：前进并视需要做角速度修正

视野内目标选择策略统一为：

- 当前视野内若有球，则选最近球
- 若两个球距离相近，则优先选右侧球，避免卡住
- 当前视野内无球，则原地搜索
- 搜索一整圈仍无球，则停止

这一部分实现于 [selectVisibleTargetByPolicy()](/home/lmy/catkin_ws/src/my_simulation/src/ball_pickup_controller.cpp:292) 与 [controlTimerCallback()](/home/lmy/catkin_ws/src/my_simulation/src/ball_pickup_controller.cpp:1192)。

## 3.3.3 传统算法 `point`

传统策略将球视为“必须尽量回到车体中线”的点目标。当前实现中，`point` 的核心角度修正函数为：

- [computePointControlHeading()](/home/lmy/catkin_ws/src/my_simulation/src/ball_pickup_controller.cpp:482)

它本质上调用通用的 [computeBandControlHeading()](/home/lmy/catkin_ws/src/my_simulation/src/ball_pickup_controller.cpp:455)，但把目标带宽设置为 `0`，因此控制目标退化为“将球拉回中线”。

传统策略示意：

```text
球心目标
   |
   v
车体中线
```

**图 3-3 传统策略 `point` 的点目标示意图**

对应特点：

- 更倾向先把球对准中线
- 只有横向偏差和朝向误差足够小，才更稳定地前进
- 在多球场景中容易出现较多的停-转-走行为

## 3.3.4 改进算法 `segment`

改进策略的核心思想是：**调整角度函数与传统策略不同**。在当前代码版本中，`segment` 使用独立的角度修正函数：

- [computeSegmentControlHeading()](/home/lmy/catkin_ws/src/my_simulation/src/ball_pickup_controller.cpp:490)

该函数仍基于 [computeBandControlHeading()](/home/lmy/catkin_ws/src/my_simulation/src/ball_pickup_controller.cpp:455)，但目标带宽设置为：

- `w_guidance = w_pickup - 0.0356`

其中 `0.0356 m` 来自前置实验得到的误差量，用于构造一个比最终拾取带更窄的“引导带”，使小车在进入实际拾取带之前先被引导到更稳的位置。

改进策略示意：

```text
真实拾取带:    |---------|
引导带:          |-----|
                  ^
            调整角度的目标是先进入引导带
```

**图 3-4 改进策略 `segment` 的带目标示意图**

对应特点：

- 不再直接追球心
- 角度修正目标是“让球尽快进入引导带”
- 在相同低层控制器下，理论上更容易减少多余转向

## 3.3.5 路径规划与实验场景设计

当前研究不采用全局地图路径规划，而使用“目标选择 + 局部闭环控制”的方式完成拾球。为了验证改进算法的优化作用，需要专门设计场景，使两种算法的目标顺序尽量一致，而差异主要体现在转向行为上。

因此实验场景建议分为三类：

1. 基础随机场景  
   用于验证算法能否稳定完成多球拾取，如 [scene_turn_compare_5_random.yaml](/home/lmy/catkin_ws/src/my_simulation/config/scenes/scene_turn_compare_5_random.yaml:1)

2. 转向对比场景  
   用于突出“传统策略反复对中、改进策略减少转向”的差异，如 [scene_turn_compare.yaml](/home/lmy/catkin_ws/src/my_simulation/config/scenes/scene_turn_compare.yaml:1)

3. 局部聚集场景  
   用于验证近距离球簇中两种算法的鲁棒性，如 `scene_15_clustered_half_*.yaml`

# 3.4 实验结果与分析

为了证明改进算法性能更优，实验结果不应只看“是否捡到球”，而应关注转向、路径和时间三个层面。

推荐导出并统计以下指标：

- 总拾取时间：`metrics.md` 中的 `Total pickup time`
- 总路径长度：`metrics.md` 中的 `Total traveled distance`
- 总累计转向角：`metrics.md` 中的 `Total accumulated yaw change`
- 平均绝对偏航角速度：`metrics.md` 中的 `Average absolute yaw rate`
- 控制命令转向信息：`cmd_vel.csv` 中的 `angular.z`
- 实际运动转向信息：`odom.csv` 中的姿态与角速度

为了便于论文中的结果展示，可以将上述指标整理成表 3-3。

**表 3-3 建议统计的核心对比指标**

| 指标类别 | 指标名称 | 作用 |
| --- | --- | --- |
| 时间指标 | 总拾取时间 | 反映整体任务效率 |
| 路径指标 | 总路径长度 | 反映行驶冗余程度 |
| 转向指标 | 总累计转向角 | 直接体现是否减少转向 |
| 转向指标 | 平均绝对偏航角速度 | 反映转向激烈程度 |
| 控制指标 | `cmd_vel.csv` 中 `angular.z` | 反映控制器发出的转向命令 |
| 运动指标 | `odom.csv` 中姿态与角速度 | 反映车辆实际执行效果 |

建议采用以下分析方式：

1. **单场景可视化对比**  
   对相同场景分别运行 `point` 和 `segment`，对比：
   - `screen.mp4`
   - `path.svg`
   - 终端日志中的 `state / mode / forward / lateral / control_heading`

2. **多次重复统计**  
   对每个场景重复运行多次，统计均值和标准差，以降低单次随机扰动影响。

3. **场景分层分析**  
   将结果按“稀疏场景 / 转向对比场景 / 聚集场景”分类比较，以说明改进策略在不同工况下的效果。

4. **控制命令与实际运动分离分析**  
   将 `cmd_vel.csv` 与 `odom.csv` 对照，可判断：
   - 是算法本身在频繁转向
   - 还是命令不大但车辆执行层放大了摆动

# 3.5 结果分析与讨论

从理论上讲，改进算法的主要优势不在于“更快选到球”，而在于“用更宽松的几何目标减少不必要的角度修正”。如果实验设计合理，则应该观察到以下趋势：

- `segment` 的总累计转向角小于 `point`
- `segment` 的平均绝对偏航角速度更低
- `segment` 的路径更连续，轨迹更少出现停-转-走
- 在多球连续场景中，`segment` 的总拾取时间更短

但也需要注意以下讨论点：

1. 当前实时控制仍主要基于 Gazebo 真值，而不是完全基于视觉定位输出。因此，当前结果更能说明“控制策略本身”的差异，而不是“视觉误差条件下的最终系统性能”。

2. 双目相机在近场可能出现目标出视野问题，因此本研究引入了前导流挡板和格挡捕获区判定。这说明机械结构设计与控制策略是耦合的，改进算法的性能提升并不只来自控制公式本身，也部分来自结构对近场拾球条件的改善。

3. 如果后续将控制闭环真正切换到视觉定位结果，则还需进一步研究视觉噪声、视差不稳定和近场遮挡对算法性能的影响。

4. 如果两种算法在某些场景下路径差异过大，导致“转向减少”优势不够直观，则应优先使用目标顺序相近、横向偏置适中的专用对比场景，而不是完全随机的大范围多球场景。

综上，建议在论文中将改进算法的贡献表述为：

- 通过将拾取目标从点扩展为前沿区域
- 并结合导流挡板形成的物理捕获区
- 在保持底层控制器一致的前提下，减少额外对中转向
- 提升连续多球拾取时的整体效率和路径平滑性
