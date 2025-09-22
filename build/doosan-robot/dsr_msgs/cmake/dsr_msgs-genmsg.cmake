# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dsr_msgs: 17 messages, 130 services")

set(MSG_I_FLAGS "-Idsr_msgs:/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dsr_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotError.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotError.msg" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotState.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotState.msg" "std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension:dsr_msgs/ModbusState:std_msgs/Float64MultiArray"
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStop.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStop.msg" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/JogMultiAxis.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/JogMultiAxis.msg" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/AlterMotionStream.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/AlterMotionStream.msg" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJStream.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJStream.msg" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLStream.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLStream.msg" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJStream.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJStream.msg" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLStream.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLStream.msg" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg" "std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension:std_msgs/Float64MultiArray"
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJRTStream.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJRTStream.msg" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLRTStream.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLRTStream.msg" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJRTStream.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJRTStream.msg" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLRTStream.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLRTStream.msg" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/TorqueRTStream.msg" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/TorqueRTStream.msg" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotMode.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotMode.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotMode.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotMode.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSystem.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSystem.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSystem.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSystem.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSpeedMode.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSpeedMode.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSpeedMode.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSpeedMode.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetCurrentPose.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetCurrentPose.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafeStopResetType.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafeStopResetType.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetLastAlarm.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetLastAlarm.srv" "dsr_msgs/LogAlarm"
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotState.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotState.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotControl.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotControl.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ManageAccessControl.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ManageAccessControl.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ReleaseProtectiveStop.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ReleaseProtectiveStop.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafetyMode.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafetyMode.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJoint.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJoint.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveLine.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveLine.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJointx.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJointx.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveCircle.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveCircle.srv" "std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension:std_msgs/Float64MultiArray"
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineJoint.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineJoint.srv" "std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension:std_msgs/Float64MultiArray"
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineTask.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineTask.srv" "std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension:std_msgs/Float64MultiArray"
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveBlending.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveBlending.srv" "std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension:std_msgs/Float64MultiArray"
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSpiral.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSpiral.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePeriodic.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePeriodic.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveWait.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveWait.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Jog.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Jog.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/JogMulti.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/JogMulti.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveStop.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveStop.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePause.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePause.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveResume.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveResume.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Trans.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Trans.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Fkin.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Fkin.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Ikin.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Ikin.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/IkinEx.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/IkinEx.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetRefCoord.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetRefCoord.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveHome.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveHome.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/CheckMotion.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/CheckMotion.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/ChangeOperationSpeed.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/ChangeOperationSpeed.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/EnableAlterMotion.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/EnableAlterMotion.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/AlterMotion.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/AlterMotion.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/DisableAlterMotion.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/DisableAlterMotion.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetSingularityHandling.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetSingularityHandling.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlMode.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlMode.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlSpace.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlSpace.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosj.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosj.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosj.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosj.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelj.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelj.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelj.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelj.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosx.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosx.srv" "std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension:std_msgs/Float64MultiArray"
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentToolFlangePosx.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentToolFlangePosx.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelx.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelx.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosx.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosx.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelx.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelx.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentSolutionSpace.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentSolutionSpace.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentRotm.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentRotm.srv" "std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension:std_msgs/Float64MultiArray"
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetJointTorque.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetJointTorque.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetExternalTorque.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetExternalTorque.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetToolForce.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetToolForce.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetSolutionSpace.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetSolutionSpace.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetOrientationError.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetOrientationError.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis1.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis1.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis2.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis2.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis1.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis1.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis2.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis2.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/IsDoneBoltTightening.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/IsDoneBoltTightening.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseComplianceCtrl.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseComplianceCtrl.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/TaskComplianceCtrl.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/TaskComplianceCtrl.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetStiffnessx.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetStiffnessx.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CalcCoord.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CalcCoord.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord1.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord1.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord2.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord2.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord3.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord3.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/OverwriteUserCartCoord.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/OverwriteUserCartCoord.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetUserCartCoord.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetUserCartCoord.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetDesiredForce.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetDesiredForce.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseForce.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseForce.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckPositionCondition.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckPositionCondition.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckForceCondition.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckForceCondition.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition1.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition1.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition2.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition2.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CoordTransform.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CoordTransform.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetWorkpieceWeight.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetWorkpieceWeight.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ResetWorkpieceWeight.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ResetWorkpieceWeight.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigCreateTool.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigCreateTool.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigDeleteTool.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigDeleteTool.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetCurrentTool.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetCurrentTool.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/GetCurrentTool.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/GetCurrentTool.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetToolShape.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetToolShape.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigCreateTcp.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigCreateTcp.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigDeleteTcp.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigDeleteTcp.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/SetCurrentTcp.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/SetCurrentTcp.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/GetCurrentTcp.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/GetCurrentTcp.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetToolDigitalOutput.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetToolDigitalOutput.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalOutput.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalOutput.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalInput.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalInput.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxDigitalOutput.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxDigitalOutput.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalOutput.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalOutput.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalInput.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalInput.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogInputType.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogInputType.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutputType.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutputType.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutput.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutput.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxAnalogInput.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxAnalogInput.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigCreateModbus.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigCreateModbus.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigDeleteModbus.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigDeleteModbus.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/SetModbusOutput.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/SetModbusOutput.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/GetModbusInput.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/GetModbusInput.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStart.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStart.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStop.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStop.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlPause.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlPause.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlResume.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlResume.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/GetDrlState.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/GetDrlState.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FClose.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FClose.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FOpen.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FOpen.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FMove.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FMove.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/SerialSendData.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/SerialSendData.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ConnectRTControl.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ConnectRTControl.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/DisconnectRTControl.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/DisconnectRTControl.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputDataList.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputDataList.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputVersionList.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputVersionList.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputDataList.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputDataList.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputVersionList.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputVersionList.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ReadDataRT.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ReadDataRT.srv" "std_msgs/MultiArrayLayout:dsr_msgs/RobotStateRT:std_msgs/Float64MultiArray:std_msgs/MultiArrayDimension"
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccJRT.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccJRT.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccXRT.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccXRT.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlInput.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlInput.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlOutput.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlOutput.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelJRT.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelJRT.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelXRT.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelXRT.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StartRTControl.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StartRTControl.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StopRTControl.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StopRTControl.srv" ""
)

get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/WriteDataRT.srv" NAME_WE)
add_custom_target(_dsr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dsr_msgs" "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/WriteDataRT.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotError.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStop.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/JogMultiAxis.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/AlterMotionStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/TorqueRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)

### Generating Services
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSystem.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSystem.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSpeedMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSpeedMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetCurrentPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafeStopResetType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetLastAlarm.srv"
  "${MSG_I_FLAGS}"
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ManageAccessControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ReleaseProtectiveStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafetyMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveLine.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJointx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveCircle.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineJoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineTask.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveBlending.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSpiral.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePeriodic.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveWait.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Jog.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/JogMulti.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePause.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveResume.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Trans.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Fkin.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Ikin.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/IkinEx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetRefCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveHome.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/CheckMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/ChangeOperationSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/EnableAlterMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/AlterMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/DisableAlterMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetSingularityHandling.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlSpace.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosx.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentToolFlangePosx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentSolutionSpace.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentRotm.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetJointTorque.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetExternalTorque.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetToolForce.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetSolutionSpace.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetOrientationError.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/IsDoneBoltTightening.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseComplianceCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/TaskComplianceCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetStiffnessx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CalcCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord3.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/OverwriteUserCartCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetUserCartCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetDesiredForce.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseForce.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckPositionCondition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckForceCondition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CoordTransform.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetWorkpieceWeight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ResetWorkpieceWeight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigCreateTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigDeleteTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetCurrentTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/GetCurrentTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetToolShape.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigCreateTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigDeleteTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/SetCurrentTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/GetCurrentTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetToolDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogInputType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutputType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxAnalogInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigCreateModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigDeleteModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/SetModbusOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/GetModbusInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStart.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlPause.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlResume.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/GetDrlState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FClose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FOpen.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FMove.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/SerialSendData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ConnectRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/DisconnectRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputDataList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputVersionList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputDataList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputVersionList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ReadDataRT.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccJRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccXRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelJRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelXRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StartRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StopRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_cpp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/WriteDataRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
)

### Generating Module File
_generate_module_cpp(dsr_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dsr_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dsr_msgs_generate_messages dsr_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotError.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStop.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/JogMultiAxis.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/AlterMotionStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/TorqueRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSystem.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSystem.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSpeedMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSpeedMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetCurrentPose.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafeStopResetType.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetLastAlarm.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotState.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ManageAccessControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ReleaseProtectiveStop.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafetyMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJoint.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveLine.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJointx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveCircle.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineJoint.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineTask.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveBlending.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSpiral.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePeriodic.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveWait.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Jog.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/JogMulti.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveStop.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePause.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveResume.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Trans.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Fkin.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Ikin.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/IkinEx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetRefCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveHome.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/CheckMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/ChangeOperationSpeed.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/EnableAlterMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/AlterMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/DisableAlterMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetSingularityHandling.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlSpace.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentToolFlangePosx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentSolutionSpace.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentRotm.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetJointTorque.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetExternalTorque.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetToolForce.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetSolutionSpace.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetOrientationError.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/IsDoneBoltTightening.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseComplianceCtrl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/TaskComplianceCtrl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetStiffnessx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CalcCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord3.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/OverwriteUserCartCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetUserCartCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetDesiredForce.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseForce.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckPositionCondition.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckForceCondition.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CoordTransform.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetWorkpieceWeight.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ResetWorkpieceWeight.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigCreateTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigDeleteTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetCurrentTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/GetCurrentTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetToolShape.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigCreateTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigDeleteTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/SetCurrentTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/GetCurrentTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetToolDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogInputType.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutputType.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxAnalogInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigCreateModbus.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigDeleteModbus.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/SetModbusOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/GetModbusInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStart.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStop.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlPause.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlResume.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/GetDrlState.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FClose.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FOpen.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FMove.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/SerialSendData.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ConnectRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/DisconnectRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputDataList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputVersionList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputDataList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputVersionList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ReadDataRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccJRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccXRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelJRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelXRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StartRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StopRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/WriteDataRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_cpp _dsr_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dsr_msgs_gencpp)
add_dependencies(dsr_msgs_gencpp dsr_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dsr_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotError.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStop.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/JogMultiAxis.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/AlterMotionStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_msg_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/TorqueRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)

### Generating Services
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSystem.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSystem.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSpeedMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSpeedMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetCurrentPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafeStopResetType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetLastAlarm.srv"
  "${MSG_I_FLAGS}"
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ManageAccessControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ReleaseProtectiveStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafetyMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveLine.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJointx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveCircle.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineJoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineTask.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveBlending.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSpiral.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePeriodic.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveWait.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Jog.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/JogMulti.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePause.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveResume.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Trans.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Fkin.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Ikin.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/IkinEx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetRefCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveHome.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/CheckMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/ChangeOperationSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/EnableAlterMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/AlterMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/DisableAlterMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetSingularityHandling.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlSpace.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosx.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentToolFlangePosx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentSolutionSpace.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentRotm.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetJointTorque.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetExternalTorque.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetToolForce.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetSolutionSpace.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetOrientationError.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/IsDoneBoltTightening.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseComplianceCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/TaskComplianceCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetStiffnessx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CalcCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord3.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/OverwriteUserCartCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetUserCartCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetDesiredForce.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseForce.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckPositionCondition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckForceCondition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CoordTransform.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetWorkpieceWeight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ResetWorkpieceWeight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigCreateTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigDeleteTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetCurrentTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/GetCurrentTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetToolShape.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigCreateTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigDeleteTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/SetCurrentTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/GetCurrentTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetToolDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogInputType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutputType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxAnalogInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigCreateModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigDeleteModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/SetModbusOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/GetModbusInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStart.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlPause.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlResume.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/GetDrlState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FClose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FOpen.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FMove.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/SerialSendData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ConnectRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/DisconnectRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputDataList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputVersionList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputDataList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputVersionList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ReadDataRT.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccJRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccXRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelJRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelXRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StartRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StopRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)
_generate_srv_eus(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/WriteDataRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
)

### Generating Module File
_generate_module_eus(dsr_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dsr_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dsr_msgs_generate_messages dsr_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotError.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStop.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/JogMultiAxis.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/AlterMotionStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/TorqueRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSystem.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSystem.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSpeedMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSpeedMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetCurrentPose.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafeStopResetType.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetLastAlarm.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotState.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ManageAccessControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ReleaseProtectiveStop.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafetyMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJoint.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveLine.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJointx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveCircle.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineJoint.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineTask.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveBlending.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSpiral.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePeriodic.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveWait.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Jog.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/JogMulti.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveStop.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePause.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveResume.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Trans.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Fkin.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Ikin.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/IkinEx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetRefCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveHome.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/CheckMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/ChangeOperationSpeed.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/EnableAlterMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/AlterMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/DisableAlterMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetSingularityHandling.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlSpace.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentToolFlangePosx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentSolutionSpace.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentRotm.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetJointTorque.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetExternalTorque.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetToolForce.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetSolutionSpace.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetOrientationError.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/IsDoneBoltTightening.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseComplianceCtrl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/TaskComplianceCtrl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetStiffnessx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CalcCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord3.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/OverwriteUserCartCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetUserCartCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetDesiredForce.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseForce.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckPositionCondition.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckForceCondition.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CoordTransform.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetWorkpieceWeight.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ResetWorkpieceWeight.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigCreateTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigDeleteTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetCurrentTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/GetCurrentTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetToolShape.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigCreateTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigDeleteTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/SetCurrentTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/GetCurrentTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetToolDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogInputType.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutputType.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxAnalogInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigCreateModbus.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigDeleteModbus.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/SetModbusOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/GetModbusInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStart.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStop.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlPause.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlResume.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/GetDrlState.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FClose.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FOpen.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FMove.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/SerialSendData.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ConnectRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/DisconnectRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputDataList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputVersionList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputDataList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputVersionList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ReadDataRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccJRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccXRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelJRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelXRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StartRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StopRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/WriteDataRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_eus _dsr_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dsr_msgs_geneus)
add_dependencies(dsr_msgs_geneus dsr_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dsr_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotError.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStop.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/JogMultiAxis.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/AlterMotionStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_msg_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/TorqueRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)

### Generating Services
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSystem.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSystem.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSpeedMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSpeedMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetCurrentPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafeStopResetType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetLastAlarm.srv"
  "${MSG_I_FLAGS}"
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ManageAccessControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ReleaseProtectiveStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafetyMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveLine.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJointx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveCircle.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineJoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineTask.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveBlending.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSpiral.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePeriodic.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveWait.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Jog.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/JogMulti.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePause.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveResume.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Trans.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Fkin.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Ikin.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/IkinEx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetRefCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveHome.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/CheckMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/ChangeOperationSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/EnableAlterMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/AlterMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/DisableAlterMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetSingularityHandling.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlSpace.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosx.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentToolFlangePosx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentSolutionSpace.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentRotm.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetJointTorque.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetExternalTorque.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetToolForce.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetSolutionSpace.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetOrientationError.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/IsDoneBoltTightening.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseComplianceCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/TaskComplianceCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetStiffnessx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CalcCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord3.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/OverwriteUserCartCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetUserCartCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetDesiredForce.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseForce.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckPositionCondition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckForceCondition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CoordTransform.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetWorkpieceWeight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ResetWorkpieceWeight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigCreateTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigDeleteTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetCurrentTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/GetCurrentTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetToolShape.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigCreateTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigDeleteTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/SetCurrentTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/GetCurrentTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetToolDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogInputType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutputType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxAnalogInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigCreateModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigDeleteModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/SetModbusOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/GetModbusInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStart.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlPause.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlResume.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/GetDrlState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FClose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FOpen.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FMove.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/SerialSendData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ConnectRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/DisconnectRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputDataList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputVersionList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputDataList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputVersionList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ReadDataRT.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccJRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccXRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelJRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelXRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StartRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StopRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)
_generate_srv_lisp(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/WriteDataRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
)

### Generating Module File
_generate_module_lisp(dsr_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dsr_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dsr_msgs_generate_messages dsr_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotError.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStop.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/JogMultiAxis.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/AlterMotionStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/TorqueRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSystem.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSystem.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSpeedMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSpeedMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetCurrentPose.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafeStopResetType.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetLastAlarm.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotState.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ManageAccessControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ReleaseProtectiveStop.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafetyMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJoint.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveLine.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJointx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveCircle.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineJoint.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineTask.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveBlending.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSpiral.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePeriodic.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveWait.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Jog.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/JogMulti.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveStop.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePause.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveResume.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Trans.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Fkin.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Ikin.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/IkinEx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetRefCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveHome.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/CheckMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/ChangeOperationSpeed.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/EnableAlterMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/AlterMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/DisableAlterMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetSingularityHandling.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlSpace.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentToolFlangePosx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentSolutionSpace.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentRotm.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetJointTorque.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetExternalTorque.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetToolForce.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetSolutionSpace.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetOrientationError.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/IsDoneBoltTightening.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseComplianceCtrl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/TaskComplianceCtrl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetStiffnessx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CalcCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord3.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/OverwriteUserCartCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetUserCartCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetDesiredForce.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseForce.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckPositionCondition.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckForceCondition.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CoordTransform.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetWorkpieceWeight.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ResetWorkpieceWeight.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigCreateTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigDeleteTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetCurrentTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/GetCurrentTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetToolShape.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigCreateTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigDeleteTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/SetCurrentTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/GetCurrentTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetToolDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogInputType.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutputType.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxAnalogInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigCreateModbus.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigDeleteModbus.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/SetModbusOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/GetModbusInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStart.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStop.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlPause.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlResume.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/GetDrlState.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FClose.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FOpen.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FMove.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/SerialSendData.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ConnectRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/DisconnectRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputDataList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputVersionList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputDataList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputVersionList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ReadDataRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccJRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccXRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelJRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelXRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StartRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StopRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/WriteDataRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_lisp _dsr_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dsr_msgs_genlisp)
add_dependencies(dsr_msgs_genlisp dsr_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dsr_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotError.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStop.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/JogMultiAxis.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/AlterMotionStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_msg_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/TorqueRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)

### Generating Services
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSystem.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSystem.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSpeedMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSpeedMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetCurrentPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafeStopResetType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetLastAlarm.srv"
  "${MSG_I_FLAGS}"
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ManageAccessControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ReleaseProtectiveStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafetyMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveLine.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJointx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveCircle.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineJoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineTask.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveBlending.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSpiral.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePeriodic.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveWait.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Jog.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/JogMulti.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePause.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveResume.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Trans.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Fkin.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Ikin.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/IkinEx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetRefCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveHome.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/CheckMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/ChangeOperationSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/EnableAlterMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/AlterMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/DisableAlterMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetSingularityHandling.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlSpace.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosx.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentToolFlangePosx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentSolutionSpace.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentRotm.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetJointTorque.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetExternalTorque.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetToolForce.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetSolutionSpace.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetOrientationError.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/IsDoneBoltTightening.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseComplianceCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/TaskComplianceCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetStiffnessx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CalcCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord3.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/OverwriteUserCartCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetUserCartCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetDesiredForce.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseForce.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckPositionCondition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckForceCondition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CoordTransform.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetWorkpieceWeight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ResetWorkpieceWeight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigCreateTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigDeleteTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetCurrentTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/GetCurrentTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetToolShape.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigCreateTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigDeleteTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/SetCurrentTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/GetCurrentTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetToolDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogInputType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutputType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxAnalogInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigCreateModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigDeleteModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/SetModbusOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/GetModbusInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStart.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlPause.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlResume.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/GetDrlState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FClose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FOpen.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FMove.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/SerialSendData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ConnectRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/DisconnectRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputDataList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputVersionList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputDataList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputVersionList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ReadDataRT.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccJRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccXRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelJRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelXRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StartRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StopRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)
_generate_srv_nodejs(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/WriteDataRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
)

### Generating Module File
_generate_module_nodejs(dsr_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dsr_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dsr_msgs_generate_messages dsr_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotError.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStop.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/JogMultiAxis.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/AlterMotionStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/TorqueRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSystem.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSystem.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSpeedMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSpeedMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetCurrentPose.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafeStopResetType.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetLastAlarm.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotState.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ManageAccessControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ReleaseProtectiveStop.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafetyMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJoint.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveLine.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJointx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveCircle.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineJoint.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineTask.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveBlending.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSpiral.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePeriodic.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveWait.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Jog.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/JogMulti.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveStop.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePause.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveResume.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Trans.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Fkin.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Ikin.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/IkinEx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetRefCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveHome.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/CheckMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/ChangeOperationSpeed.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/EnableAlterMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/AlterMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/DisableAlterMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetSingularityHandling.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlSpace.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentToolFlangePosx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentSolutionSpace.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentRotm.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetJointTorque.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetExternalTorque.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetToolForce.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetSolutionSpace.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetOrientationError.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/IsDoneBoltTightening.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseComplianceCtrl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/TaskComplianceCtrl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetStiffnessx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CalcCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord3.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/OverwriteUserCartCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetUserCartCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetDesiredForce.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseForce.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckPositionCondition.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckForceCondition.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CoordTransform.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetWorkpieceWeight.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ResetWorkpieceWeight.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigCreateTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigDeleteTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetCurrentTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/GetCurrentTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetToolShape.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigCreateTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigDeleteTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/SetCurrentTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/GetCurrentTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetToolDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogInputType.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutputType.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxAnalogInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigCreateModbus.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigDeleteModbus.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/SetModbusOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/GetModbusInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStart.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStop.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlPause.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlResume.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/GetDrlState.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FClose.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FOpen.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FMove.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/SerialSendData.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ConnectRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/DisconnectRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputDataList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputVersionList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputDataList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputVersionList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ReadDataRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccJRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccXRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelJRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelXRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StartRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StopRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/WriteDataRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_nodejs _dsr_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dsr_msgs_gennodejs)
add_dependencies(dsr_msgs_gennodejs dsr_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dsr_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotError.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStop.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/JogMultiAxis.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/AlterMotionStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_msg_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/TorqueRTStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)

### Generating Services
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSystem.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSystem.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSpeedMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSpeedMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetCurrentPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafeStopResetType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetLastAlarm.srv"
  "${MSG_I_FLAGS}"
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ManageAccessControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ReleaseProtectiveStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafetyMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveLine.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJointx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveCircle.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineJoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineTask.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveBlending.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSpiral.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePeriodic.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveWait.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Jog.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/JogMulti.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePause.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveResume.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Trans.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Fkin.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Ikin.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/IkinEx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetRefCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveHome.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/CheckMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/ChangeOperationSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/EnableAlterMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/AlterMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/DisableAlterMotion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetSingularityHandling.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlSpace.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosx.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentToolFlangePosx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentSolutionSpace.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentRotm.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetJointTorque.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetExternalTorque.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetToolForce.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetSolutionSpace.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetOrientationError.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/IsDoneBoltTightening.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseComplianceCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/TaskComplianceCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetStiffnessx.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CalcCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord3.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/OverwriteUserCartCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetUserCartCoord.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetDesiredForce.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseForce.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckPositionCondition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckForceCondition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition2.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CoordTransform.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetWorkpieceWeight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ResetWorkpieceWeight.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigCreateTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigDeleteTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetCurrentTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/GetCurrentTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetToolShape.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigCreateTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigDeleteTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/SetCurrentTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/GetCurrentTcp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetToolDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogInputType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutputType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxAnalogInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigCreateModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigDeleteModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/SetModbusOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/GetModbusInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStart.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlPause.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlResume.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/GetDrlState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FClose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FOpen.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FMove.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/SerialSendData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ConnectRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/DisconnectRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputDataList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputVersionList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputDataList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputVersionList.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ReadDataRT.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccJRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccXRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlInput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelJRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelXRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StartRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StopRTControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)
_generate_srv_py(dsr_msgs
  "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/WriteDataRT.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
)

### Generating Module File
_generate_module_py(dsr_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dsr_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dsr_msgs_generate_messages dsr_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/LogAlarm.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ModbusState.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotError.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStop.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/JogMultiAxis.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/AlterMotionStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/RobotStateRT.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoJRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/ServoLRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedJRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/SpeedLRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/msg/TorqueRTStream.msg" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSystem.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSystem.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotSpeedMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotSpeedMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetCurrentPose.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafeStopResetType.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetLastAlarm.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/GetRobotState.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetRobotControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ManageAccessControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/ReleaseProtectiveStop.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./system/SetSafetyMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJoint.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveLine.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveJointx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveCircle.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineJoint.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSplineTask.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveBlending.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveSpiral.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePeriodic.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveWait.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Jog.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/JogMulti.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveStop.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MovePause.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveResume.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Trans.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Fkin.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/Ikin.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/IkinEx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetRefCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/MoveHome.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/CheckMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/ChangeOperationSpeed.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/EnableAlterMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/AlterMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/DisableAlterMotion.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./motion/SetSingularityHandling.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlMode.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetControlSpace.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelj.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentPosx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentToolFlangePosx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentVelx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredPosx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetDesiredVelx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentSolutionSpace.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetCurrentRotm.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetJointTorque.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetExternalTorque.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetToolForce.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetSolutionSpace.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./aux_control/GetOrientationError.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ParallelAxis2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/AlignAxis2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/IsDoneBoltTightening.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseComplianceCtrl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/TaskComplianceCtrl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetStiffnessx.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CalcCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetUserCartCoord3.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/OverwriteUserCartCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetUserCartCoord.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/SetDesiredForce.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ReleaseForce.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckPositionCondition.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckForceCondition.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition1.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CheckOrientationCondition2.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/CoordTransform.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/GetWorkpieceWeight.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./force/ResetWorkpieceWeight.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigCreateTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/ConfigDeleteTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetCurrentTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/GetCurrentTool.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tool/SetToolShape.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigCreateTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/ConfigDeleteTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/SetCurrentTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./tcp/GetCurrentTcp.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetToolDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetToolDigitalInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxDigitalInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogInputType.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutputType.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/SetCtrlBoxAnalogOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./io/GetCtrlBoxAnalogInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigCreateModbus.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/ConfigDeleteModbus.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/SetModbusOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./modbus/GetModbusInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStart.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlStop.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlPause.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/DrlResume.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./drl/GetDrlState.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FClose.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FOpen.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/Robotiq2FMove.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./gripper/SerialSendData.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ConnectRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/DisconnectRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputDataList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlInputVersionList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputDataList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/GetRTControlOutputVersionList.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/ReadDataRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccJRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetAccXRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlInput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetRTControlOutput.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelJRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/SetVelXRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StartRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/StopRTControl.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vinh/catkin_ws/src/doosan-robot/dsr_msgs/srv/./realtime/WriteDataRT.srv" NAME_WE)
add_dependencies(dsr_msgs_generate_messages_py _dsr_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dsr_msgs_genpy)
add_dependencies(dsr_msgs_genpy dsr_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dsr_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dsr_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dsr_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dsr_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dsr_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dsr_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dsr_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dsr_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dsr_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dsr_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dsr_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
