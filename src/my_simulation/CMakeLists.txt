# 1. 声明要求的最低 CMake 版本 (提升至 3.10 以更好支持 C++14)
cmake_minimum_required(VERSION 3.10)

# 2. 定义 ROS 包名称 (必须与 package.xml 保持一致)
project(my_simulation)

# 3. 强制要求使用 C++14 或更高标准（OpenCV DNN 模块推荐）
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 4. 寻找 catkin 依赖
# (加入了处理 ROS 图像消息所需的 sensor_msgs, cv_bridge, image_transport)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  gazebo_msgs
  std_srvs
  sensor_msgs
  image_transport
)

# 5. 寻找系统环境中的 OpenCV 库
find_package(OpenCV REQUIRED)

# 5.1 导出 catkin 包信息
catkin_package()

# 6. 包含所有依赖的头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 7. 生成可执行文件
# 节点名称定义为 stereo_node，指向 src 目录下的源码
add_executable(stereo_node src/stereo_vision_localization.cpp)
add_executable(ball_pickup_controller src/ball_pickup_controller.cpp)

# 8. 将 ROS 通信库与 OpenCV 动态链接库合并链接到该节点
target_link_libraries(stereo_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(ball_pickup_controller
  ${catkin_LIBRARIES}
)

install(TARGETS stereo_node ball_pickup_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/ball_scene_spawner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
