# Install script for directory: /home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/haoyu/rpi4_11_ws/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nlink_parser/msg" TYPE FILE FILES
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackAnchorframe0.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackNode0.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackNode1.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackNode2.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackNodeframe0.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackNodeframe1.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackNodeframe2.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackNodeframe3.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackTag.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackTagframe0.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/TofsenseCascade.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/TofsenseFrame0.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackAoaNode0.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackAoaNodeframe0.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackNode4Anchor.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackNode4Tag.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackNodeframe4.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackNode5.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackNodeframe5.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackNode6.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/LinktrackNodeframe6.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/TofsenseMFrame0.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/TofsenseMFrame0Pixel.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/IotFrame0.msg"
    "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/msg/IotFrame0Node.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nlink_parser/cmake" TYPE FILE FILES "/home/haoyu/rpi4_11_ws/build/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/catkin_generated/installspace/nlink_parser-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/haoyu/rpi4_11_ws/devel/include/nlink_parser")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/haoyu/rpi4_11_ws/devel/share/roseus/ros/nlink_parser")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/haoyu/rpi4_11_ws/devel/share/common-lisp/ros/nlink_parser")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/haoyu/rpi4_11_ws/devel/share/gennodejs/ros/nlink_parser")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/haoyu/rpi4_11_ws/devel/lib/python3/dist-packages/nlink_parser")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/haoyu/rpi4_11_ws/devel/lib/python3/dist-packages/nlink_parser")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/haoyu/rpi4_11_ws/build/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/catkin_generated/installspace/nlink_parser.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nlink_parser/cmake" TYPE FILE FILES "/home/haoyu/rpi4_11_ws/build/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/catkin_generated/installspace/nlink_parser-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nlink_parser/cmake" TYPE FILE FILES
    "/home/haoyu/rpi4_11_ws/build/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/catkin_generated/installspace/nlink_parserConfig.cmake"
    "/home/haoyu/rpi4_11_ws/build/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/catkin_generated/installspace/nlink_parserConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nlink_parser" TYPE FILE FILES "/home/haoyu/rpi4_11_ws/src/ds5_uwb_remote_control/nlink_uwb_tools/nlink_parser/package.xml")
endif()

