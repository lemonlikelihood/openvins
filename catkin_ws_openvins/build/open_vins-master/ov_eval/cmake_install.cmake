# Install script for directory: /home/SENSETIME/yuanjin/Workspace/catkin_ws_openvins/src/open_vins-master/ov_eval

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/SENSETIME/yuanjin/Workspace/catkin_ws_openvins/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/SENSETIME/yuanjin/Workspace/catkin_ws_openvins/build/open_vins-master/ov_eval/catkin_generated/installspace/ov_eval.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ov_eval/cmake" TYPE FILE FILES
    "/home/SENSETIME/yuanjin/Workspace/catkin_ws_openvins/build/open_vins-master/ov_eval/catkin_generated/installspace/ov_evalConfig.cmake"
    "/home/SENSETIME/yuanjin/Workspace/catkin_ws_openvins/build/open_vins-master/ov_eval/catkin_generated/installspace/ov_evalConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ov_eval" TYPE FILE FILES "/home/SENSETIME/yuanjin/Workspace/catkin_ws_openvins/src/open_vins-master/ov_eval/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ov_eval" TYPE PROGRAM FILES "/home/SENSETIME/yuanjin/Workspace/catkin_ws_openvins/build/open_vins-master/ov_eval/catkin_generated/installspace/pid_ros.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ov_eval" TYPE PROGRAM FILES "/home/SENSETIME/yuanjin/Workspace/catkin_ws_openvins/build/open_vins-master/ov_eval/catkin_generated/installspace/pid_sys.py")
endif()
