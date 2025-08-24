# Install script for directory: /aichallenge/workspace/src/aichallenge_submit/CppAD/example

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/aichallenge/install/cppad")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "NOTFOUND")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/aichallenge/build/cppad/example/abs_normal/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/atomic_four/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/atomic_three/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/atomic_two/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/chkpoint_two/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/compare_change/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/general/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/get_started/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/graph/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/json/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/multi_thread/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/optimize/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/print_for/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/sparse/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/utility/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/valvector/cmake_install.cmake")
  include("/aichallenge/build/cppad/example/jit/cmake_install.cmake")

endif()

