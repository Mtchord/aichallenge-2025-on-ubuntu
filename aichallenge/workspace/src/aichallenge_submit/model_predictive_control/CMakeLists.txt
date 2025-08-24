cmake_minimum_required(VERSION 3.8)
project(model_predictive_control)

add_compile_options(-Wno-implicit-fallthrough -Wno-unused-parameter -Wno-error=empty-body -Wno-empty-body)

include_directories(
  # /usr/include
  # /usr/local/include
  # /usr/local/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../CppAD/include
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  # ${IPOPT_INCLUDE_DIRS}
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/Ipopt/src/.libs   # ← libipopt.so のある場所
)

find_package(PkgConfig REQUIRED)
# pkg_check_modules(IPOPT REQUIRED ipopt)

# include_directories(${IPOPT_INCLUDE_DIRS})
# link_directories(${IPOPT_LIBRARY_DIRS})

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

find_library(IPOPT_LIB ipopt
  HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../Ipopt/src/.libs
  NO_DEFAULT_PATH
)

ament_auto_add_executable(model_predictive_control
  src/model_predictive_control.cpp
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../Ipopt/src/.libs)

target_link_libraries(model_predictive_control
  ${IPOPT_LIB}
)

ament_auto_package()
