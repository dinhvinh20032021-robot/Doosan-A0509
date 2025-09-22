;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ReadDataRT)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ReadDataRT (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::READDATART")
  (make-package "DSR_MSGS::READDATART"))
(unless (find-package "DSR_MSGS::READDATARTREQUEST")
  (make-package "DSR_MSGS::READDATARTREQUEST"))
(unless (find-package "DSR_MSGS::READDATARTRESPONSE")
  (make-package "DSR_MSGS::READDATARTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::ReadDataRTRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::ReadDataRTRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass dsr_msgs::ReadDataRTResponse
  :super ros::object
  :slots (_data ))

(defmethod dsr_msgs::ReadDataRTResponse
  (:init
   (&key
    ((:data __data) (instance dsr_msgs::RobotStateRT :init))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; dsr_msgs/RobotStateRT _data
    (send _data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; dsr_msgs/RobotStateRT _data
       (send _data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; dsr_msgs/RobotStateRT _data
     (send _data :deserialize buf ptr-) (incf ptr- (send _data :serialization-length))
   ;;
   self)
  )

(defclass dsr_msgs::ReadDataRT
  :super ros::object
  :slots ())

(setf (get dsr_msgs::ReadDataRT :md5sum-) "1e1cb5234a76f1e708ea68842792bed5")
(setf (get dsr_msgs::ReadDataRT :datatype-) "dsr_msgs/ReadDataRT")
(setf (get dsr_msgs::ReadDataRT :request) dsr_msgs::ReadDataRTRequest)
(setf (get dsr_msgs::ReadDataRT :response) dsr_msgs::ReadDataRTResponse)

(defmethod dsr_msgs::ReadDataRTRequest
  (:response () (instance dsr_msgs::ReadDataRTResponse :init)))

(setf (get dsr_msgs::ReadDataRTRequest :md5sum-) "1e1cb5234a76f1e708ea68842792bed5")
(setf (get dsr_msgs::ReadDataRTRequest :datatype-) "dsr_msgs/ReadDataRTRequest")
(setf (get dsr_msgs::ReadDataRTRequest :definition-)
      "#____________________________________________________________________________________________
# read_data_rt
#____________________________________________________________________________________________

---
RobotStateRT       data

================================================================================
MSG: dsr_msgs/RobotStateRT
# timestamp at the data of data acquisition
float64                      time_stamp
# actual joint position from incremental encoder at motor side(used for control) [deg]
float64[6]                   actual_joint_position
# actual joint position from absolute encoder at link side (used for exact link position) [deg]
float64[6]                   actual_joint_position_abs
# actual joint velocity from incremental encoder at motor side [deg/s]
float64[6]                   actual_joint_velocity
# actual joint velocity from absolute encoder at link side [deg/s]
float64[6]                   actual_joint_velocity_abs
# actual robot tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg]
float64[6]                   actual_tcp_position
# actual robot tcp velocity w.r.t. base coordinates [mm, deg/s]
float64[6]                   actual_tcp_velocity
# actual robot flange position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg]
float64[6]                   actual_flange_position
# robot flange velocity w.r.t. base coordinates [mm, deg/s]
float64[6]                   actual_flange_velocity
# actual motor torque applying gear ratio = gear_ratio * current2torque_constant * motor current [Nm]
float64[6]                   actual_motor_torque
# estimated joint torque by robot controller [Nm]
float64[6]                   actual_joint_torque
# calibrated joint torque sensor data [Nm]
float64[6]                   raw_joint_torque
# calibrated force torque sensor data w.r.t. flange coordinates [N, Nm]
float64[6]                   raw_force_torque
# estimated external joint torque [Nm]
float64[6]                   external_joint_torque
# estimated tcp force w.r.t. base coordinates [N, Nm] 
float64[6]                   external_tcp_force
# target joint position [deg]
float64[6]                   target_joint_position
# target joint velocity [deg/s]
float64[6]                   target_joint_velocity
# target joint acceleration [deg/s^2] 
float64[6]                   target_joint_acceleration
# target motor torque [Nm] 
float64[6]                   target_motor_torque
# target tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg] 
float64[6]                   target_tcp_position
# target tcp velocity w.r.t. base coordinates [mm, deg/s]
float64[6]                   target_tcp_velocity
# jacobian matrix=J(q) w.r.t. base coordinates
std_msgs/Float64MultiArray[] jacobian_matrix
# gravity torque=g(q) [Nm]
float64[6]                   gravity_torque
# coriolis matrix=C(q,q_dot)  [6][6]
std_msgs/Float64MultiArray[] coriolis_matrix
# mass matrix=M(q) [6][6]
std_msgs/Float64MultiArray[] mass_matrix
# robot configuration 
uint16                       solution_space
# minimum singular value 
float64                      singularity
# current operation speed rate(1~100 %) 
float64                      operation_speed_rate
# joint temperature(celsius) 
float64[6]                   joint_temperature
# controller digital input(16 channel) 
uint16                       controller_digital_input
# controller digital output(16 channel) 
uint16                       controller_digital_output
# controller analog input type(2 channel) 
uint8[2]                      controller_analog_input_type
# controller analog input(2 channel) 
float64[2]                   controller_analog_input
# controller analog output type(2 channel) 
uint8[2]                     controller_analog_output_type
# controller analog output(2 channel) 
float64[2]                   controller_analog_output
# flange digital input(A-Series: 2 channel, M/H-Series: 6 channel) 
uint8                        flange_digital_input
# flange digital output(A-Series: 2 channel, M/H-Series: 6 channel) 
uint8                        flange_digital_output
# flange analog input(A-Series: 2 channel, M/H-Series: 4 channel) 
float64[4]                   flange_analog_input
# strobe count(increased by 1 when detecting setting edge) 
uint8[2]                     external_encoder_strobe_count
# external encoder count 
uint16[2]                    external_encoder_count
# final goal joint position (reserved) 
float64[6]                   goal_joint_position
# final goal tcp position (reserved) 
float64[6]                   goal_tcp_position
# ROBOT_MODE_MANUAL(0), ROBOT_MODE_AUTONOMOUS(1), ROBOT_MODE_MEASURE(2) 
uint8                        robot_mode
# STATE_INITIALIZING(0), STATE_STANDBY(1), STATE_MOVING(2), STATE_SAFE_OFF(3), STATE_TEACHING(4), STATE_SAFE_STOP(5), STATE_EMERGENCY_STOP, STATE_HOMMING, STATE_RECOVERY, STATE_SAFE_STOP2, STATE_SAFE_OFF2, 
uint8                        robot_state
# position control mode, torque mode 
uint16                       control_mode
# Reserved 
uint8[256]                   reserved
================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
")

(setf (get dsr_msgs::ReadDataRTResponse :md5sum-) "1e1cb5234a76f1e708ea68842792bed5")
(setf (get dsr_msgs::ReadDataRTResponse :datatype-) "dsr_msgs/ReadDataRTResponse")
(setf (get dsr_msgs::ReadDataRTResponse :definition-)
      "#____________________________________________________________________________________________
# read_data_rt
#____________________________________________________________________________________________

---
RobotStateRT       data

================================================================================
MSG: dsr_msgs/RobotStateRT
# timestamp at the data of data acquisition
float64                      time_stamp
# actual joint position from incremental encoder at motor side(used for control) [deg]
float64[6]                   actual_joint_position
# actual joint position from absolute encoder at link side (used for exact link position) [deg]
float64[6]                   actual_joint_position_abs
# actual joint velocity from incremental encoder at motor side [deg/s]
float64[6]                   actual_joint_velocity
# actual joint velocity from absolute encoder at link side [deg/s]
float64[6]                   actual_joint_velocity_abs
# actual robot tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg]
float64[6]                   actual_tcp_position
# actual robot tcp velocity w.r.t. base coordinates [mm, deg/s]
float64[6]                   actual_tcp_velocity
# actual robot flange position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg]
float64[6]                   actual_flange_position
# robot flange velocity w.r.t. base coordinates [mm, deg/s]
float64[6]                   actual_flange_velocity
# actual motor torque applying gear ratio = gear_ratio * current2torque_constant * motor current [Nm]
float64[6]                   actual_motor_torque
# estimated joint torque by robot controller [Nm]
float64[6]                   actual_joint_torque
# calibrated joint torque sensor data [Nm]
float64[6]                   raw_joint_torque
# calibrated force torque sensor data w.r.t. flange coordinates [N, Nm]
float64[6]                   raw_force_torque
# estimated external joint torque [Nm]
float64[6]                   external_joint_torque
# estimated tcp force w.r.t. base coordinates [N, Nm] 
float64[6]                   external_tcp_force
# target joint position [deg]
float64[6]                   target_joint_position
# target joint velocity [deg/s]
float64[6]                   target_joint_velocity
# target joint acceleration [deg/s^2] 
float64[6]                   target_joint_acceleration
# target motor torque [Nm] 
float64[6]                   target_motor_torque
# target tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg] 
float64[6]                   target_tcp_position
# target tcp velocity w.r.t. base coordinates [mm, deg/s]
float64[6]                   target_tcp_velocity
# jacobian matrix=J(q) w.r.t. base coordinates
std_msgs/Float64MultiArray[] jacobian_matrix
# gravity torque=g(q) [Nm]
float64[6]                   gravity_torque
# coriolis matrix=C(q,q_dot)  [6][6]
std_msgs/Float64MultiArray[] coriolis_matrix
# mass matrix=M(q) [6][6]
std_msgs/Float64MultiArray[] mass_matrix
# robot configuration 
uint16                       solution_space
# minimum singular value 
float64                      singularity
# current operation speed rate(1~100 %) 
float64                      operation_speed_rate
# joint temperature(celsius) 
float64[6]                   joint_temperature
# controller digital input(16 channel) 
uint16                       controller_digital_input
# controller digital output(16 channel) 
uint16                       controller_digital_output
# controller analog input type(2 channel) 
uint8[2]                      controller_analog_input_type
# controller analog input(2 channel) 
float64[2]                   controller_analog_input
# controller analog output type(2 channel) 
uint8[2]                     controller_analog_output_type
# controller analog output(2 channel) 
float64[2]                   controller_analog_output
# flange digital input(A-Series: 2 channel, M/H-Series: 6 channel) 
uint8                        flange_digital_input
# flange digital output(A-Series: 2 channel, M/H-Series: 6 channel) 
uint8                        flange_digital_output
# flange analog input(A-Series: 2 channel, M/H-Series: 4 channel) 
float64[4]                   flange_analog_input
# strobe count(increased by 1 when detecting setting edge) 
uint8[2]                     external_encoder_strobe_count
# external encoder count 
uint16[2]                    external_encoder_count
# final goal joint position (reserved) 
float64[6]                   goal_joint_position
# final goal tcp position (reserved) 
float64[6]                   goal_tcp_position
# ROBOT_MODE_MANUAL(0), ROBOT_MODE_AUTONOMOUS(1), ROBOT_MODE_MEASURE(2) 
uint8                        robot_mode
# STATE_INITIALIZING(0), STATE_STANDBY(1), STATE_MOVING(2), STATE_SAFE_OFF(3), STATE_TEACHING(4), STATE_SAFE_STOP(5), STATE_EMERGENCY_STOP, STATE_HOMMING, STATE_RECOVERY, STATE_SAFE_STOP2, STATE_SAFE_OFF2, 
uint8                        robot_state
# position control mode, torque mode 
uint16                       control_mode
# Reserved 
uint8[256]                   reserved
================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
")



(provide :dsr_msgs/ReadDataRT "1e1cb5234a76f1e708ea68842792bed5")


