cmake_minimum_required(VERSION 3.8)
project(model_predictive_control)

include_directories(
  /usr/include
  /usr/local/include
)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(model_predictive_control
  src/model_predictive_control.cpp
)

ament_auto_package()
