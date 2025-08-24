# Install script for directory: /home/genki-yamamoto/aichallenge-2025/aichallenge/workspace/src/aichallenge_submit/CppAD/cppad_lib

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/genki-yamamoto/aichallenge-2025/install/cppad")
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/genki-yamamoto/aichallenge-2025/install/cppad/lib/libcppad_lib.so.2409" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/genki-yamamoto/aichallenge-2025/install/cppad/lib/libcppad_lib.so.2409")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/genki-yamamoto/aichallenge-2025/install/cppad/lib/libcppad_lib.so.2409"
         RPATH "/home/genki-yamamoto/aichallenge-2025/install/cppad/lib")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/genki-yamamoto/aichallenge-2025/install/cppad/lib/libcppad_lib.so.2409")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/genki-yamamoto/aichallenge-2025/install/cppad/lib" TYPE SHARED_LIBRARY FILES "/home/genki-yamamoto/aichallenge-2025/build/cppad/cppad_lib/libcppad_lib.so.2409")
  if(EXISTS "$ENV{DESTDIR}/home/genki-yamamoto/aichallenge-2025/install/cppad/lib/libcppad_lib.so.2409" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/genki-yamamoto/aichallenge-2025/install/cppad/lib/libcppad_lib.so.2409")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/genki-yamamoto/aichallenge-2025/install/cppad/lib/libcppad_lib.so.2409"
         OLD_RPATH "/home/genki-yamamoto/aichallenge-2025/build/cppad/cppad_lib:"
         NEW_RPATH "/home/genki-yamamoto/aichallenge-2025/install/cppad/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/genki-yamamoto/aichallenge-2025/install/cppad/lib/libcppad_lib.so.2409")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/genki-yamamoto/aichallenge-2025/install/cppad/lib/libcppad_lib.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/genki-yamamoto/aichallenge-2025/install/cppad/lib" TYPE SHARED_LIBRARY FILES "/home/genki-yamamoto/aichallenge-2025/build/cppad/cppad_lib/libcppad_lib.so")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/genki-yamamoto/aichallenge-2025/build/cppad/cppad_lib/CMakeFiles/cppad_lib.dir/install-cxx-module-bmi-NOTFOUND.cmake" OPTIONAL)
endif()

