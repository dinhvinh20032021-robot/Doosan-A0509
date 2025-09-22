# Install script for directory: /home/vinh/catkin_ws/src/doosan-robot/dsr_example/py

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/vinh/catkin_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/vinh/catkin_ws/build/doosan-robot/dsr_example/py/catkin_generated/installspace/dsr_example_py.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dsr_example_py/cmake" TYPE FILE FILES
    "/home/vinh/catkin_ws/build/doosan-robot/dsr_example/py/catkin_generated/installspace/dsr_example_pyConfig.cmake"
    "/home/vinh/catkin_ws/build/doosan-robot/dsr_example/py/catkin_generated/installspace/dsr_example_pyConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dsr_example_py" TYPE FILE FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/demo/dance_m1013.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/demo/joy_ds4.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/demo/joy_multi_xbox360.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/demo/joy_xbox360.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/gripper/object_pick_and_place_simple.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/gripper/pick_and_place_simple.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/gripper/real_pick_and_place_simple.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/simple/drl_reader_simple.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/simple/dsr_service_drl_simple.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/simple/dsr_service_io_simple.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/simple/dsr_service_modbus_simple.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/simple/dsr_service_motion_simple.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/simple/dsr_service_tool_simple.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/simple/dsr_simple_test.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/simple/jog_simple.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dsr_example_py" TYPE PROGRAM FILES "/home/vinh/catkin_ws/src/doosan-robot/dsr_example/py/scripts/simple/single_robot_simple.py")
endif()

