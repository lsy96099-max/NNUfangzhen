cmake_minimum_required(VERSION 3.0.2)
project(fcu_core)

## ---------------------- 编译基础配置 ----------------------
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## ---------------------- 依赖包查找 ----------------------
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  tf
  serial
  quadrotor_msgs
)
# 补全必要依赖
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

## ---------------------- 包导出配置（删除不存在的 include 目录） ----------------------
catkin_package(
  CATKIN_DEPENDS  # 只保留依赖声明，删除 INCLUDE_DIRS（无本地头文件）
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  serial
  quadrotor_msgs
)

## ---------------------- 头文件路径 ----------------------
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## ---------------------- 可执行节点编译 ----------------------
# 1. 飞控桥接节点（001~006）
add_executable(fcu_bridge_001 src/fcu_bridge_001.cpp)
target_link_libraries(fcu_bridge_001 ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(fcu_bridge_002 src/fcu_bridge_002.cpp)
target_link_libraries(fcu_bridge_002 ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(fcu_bridge_003 src/fcu_bridge_003.cpp)
target_link_libraries(fcu_bridge_003 ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(fcu_bridge_004 src/fcu_bridge_004.cpp)
target_link_libraries(fcu_bridge_004 ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(fcu_bridge_005 src/fcu_bridge_005.cpp)
target_link_libraries(fcu_bridge_005 ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(fcu_bridge_006 src/fcu_bridge_006.cpp)
target_link_libraries(fcu_bridge_006 ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

# 2. 指令输入节点
add_executable(fcu_command src/fcu_command.cpp)
target_link_libraries(fcu_command ${catkin_LIBRARIES})

# 3. 核心Mission节点（链接yaml-cpp）
add_executable(fcu_mission src/fcu_mission.cpp)
target_link_libraries(fcu_mission
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# 4. NNU仿真节点
add_executable(fcu_mission_sim src/fcu_mission_sim.cpp)
target_link_libraries(fcu_mission_sim
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

## ---------------------- 安装配置 ----------------------
install(TARGETS
  fcu_bridge_001 fcu_bridge_002 fcu_bridge_003
  fcu_bridge_004 fcu_bridge_005 fcu_bridge_006
  fcu_command fcu_mission fcu_mission_sim
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装资源文件（若有launch/rviz_cfg目录，没有则注释）
install(DIRECTORY launch rviz_cfg config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
