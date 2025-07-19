cmake_minimum_required(VERSION 3.14)
project(simple_pure_pursuit)

## コンパイル設定
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(rclcpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tier4_autoware_utils REQUIRED)
find_package(motion_utils REQUIRED)
find_package(Eigen3 REQUIRED)

## 実行可能ファイル
add_executable(${PROJECT_NAME}
  src/simple_pure_pursuit.cpp
)

# OSQP ヘッダーとライブラリパスの指定（pkg-config使わない）
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  /usr/local/include
)
target_link_directories(${PROJECT_NAME} PRIVATE
  /usr/local/lib
)
target_link_libraries(${PROJECT_NAME} osqp)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  geometry_msgs
  nav_msgs
  autoware_auto_control_msgs
  autoware_auto_planning_msgs
  tf2
  tf2_geometry_msgs
  tier4_autoware_utils
  motion_utils
  Eigen3
)


install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
