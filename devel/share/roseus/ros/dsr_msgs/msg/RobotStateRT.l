;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::RobotStateRT)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'RobotStateRT (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::ROBOTSTATERT")
  (make-package "DSR_MSGS::ROBOTSTATERT"))

(in-package "ROS")
;;//! \htmlinclude RobotStateRT.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dsr_msgs::RobotStateRT
  :super ros::object
  :slots (_time_stamp _actual_joint_position _actual_joint_position_abs _actual_joint_velocity _actual_joint_velocity_abs _actual_tcp_position _actual_tcp_velocity _actual_flange_position _actual_flange_velocity _actual_motor_torque _actual_joint_torque _raw_joint_torque _raw_force_torque _external_joint_torque _external_tcp_force _target_joint_position _target_joint_velocity _target_joint_acceleration _target_motor_torque _target_tcp_position _target_tcp_velocity _jacobian_matrix _gravity_torque _coriolis_matrix _mass_matrix _solution_space _singularity _operation_speed_rate _joint_temperature _controller_digital_input _controller_digital_output _controller_analog_input_type _controller_analog_input _controller_analog_output_type _controller_analog_output _flange_digital_input _flange_digital_output _flange_analog_input _external_encoder_strobe_count _external_encoder_count _goal_joint_position _goal_tcp_position _robot_mode _robot_state _control_mode _reserved ))

(defmethod dsr_msgs::RobotStateRT
  (:init
   (&key
    ((:time_stamp __time_stamp) 0.0)
    ((:actual_joint_position __actual_joint_position) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:actual_joint_position_abs __actual_joint_position_abs) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:actual_joint_velocity __actual_joint_velocity) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:actual_joint_velocity_abs __actual_joint_velocity_abs) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:actual_tcp_position __actual_tcp_position) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:actual_tcp_velocity __actual_tcp_velocity) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:actual_flange_position __actual_flange_position) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:actual_flange_velocity __actual_flange_velocity) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:actual_motor_torque __actual_motor_torque) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:actual_joint_torque __actual_joint_torque) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:raw_joint_torque __raw_joint_torque) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:raw_force_torque __raw_force_torque) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:external_joint_torque __external_joint_torque) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:external_tcp_force __external_tcp_force) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:target_joint_position __target_joint_position) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:target_joint_velocity __target_joint_velocity) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:target_joint_acceleration __target_joint_acceleration) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:target_motor_torque __target_motor_torque) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:target_tcp_position __target_tcp_position) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:target_tcp_velocity __target_tcp_velocity) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:jacobian_matrix __jacobian_matrix) ())
    ((:gravity_torque __gravity_torque) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:coriolis_matrix __coriolis_matrix) ())
    ((:mass_matrix __mass_matrix) ())
    ((:solution_space __solution_space) 0)
    ((:singularity __singularity) 0.0)
    ((:operation_speed_rate __operation_speed_rate) 0.0)
    ((:joint_temperature __joint_temperature) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:controller_digital_input __controller_digital_input) 0)
    ((:controller_digital_output __controller_digital_output) 0)
    ((:controller_analog_input_type __controller_analog_input_type) (make-array 2 :initial-element 0 :element-type :char))
    ((:controller_analog_input __controller_analog_input) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:controller_analog_output_type __controller_analog_output_type) (make-array 2 :initial-element 0 :element-type :char))
    ((:controller_analog_output __controller_analog_output) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:flange_digital_input __flange_digital_input) 0)
    ((:flange_digital_output __flange_digital_output) 0)
    ((:flange_analog_input __flange_analog_input) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:external_encoder_strobe_count __external_encoder_strobe_count) (make-array 2 :initial-element 0 :element-type :char))
    ((:external_encoder_count __external_encoder_count) (make-array 2 :initial-element 0 :element-type :integer))
    ((:goal_joint_position __goal_joint_position) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:goal_tcp_position __goal_tcp_position) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:robot_mode __robot_mode) 0)
    ((:robot_state __robot_state) 0)
    ((:control_mode __control_mode) 0)
    ((:reserved __reserved) (make-array 256 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _time_stamp (float __time_stamp))
   (setq _actual_joint_position __actual_joint_position)
   (setq _actual_joint_position_abs __actual_joint_position_abs)
   (setq _actual_joint_velocity __actual_joint_velocity)
   (setq _actual_joint_velocity_abs __actual_joint_velocity_abs)
   (setq _actual_tcp_position __actual_tcp_position)
   (setq _actual_tcp_velocity __actual_tcp_velocity)
   (setq _actual_flange_position __actual_flange_position)
   (setq _actual_flange_velocity __actual_flange_velocity)
   (setq _actual_motor_torque __actual_motor_torque)
   (setq _actual_joint_torque __actual_joint_torque)
   (setq _raw_joint_torque __raw_joint_torque)
   (setq _raw_force_torque __raw_force_torque)
   (setq _external_joint_torque __external_joint_torque)
   (setq _external_tcp_force __external_tcp_force)
   (setq _target_joint_position __target_joint_position)
   (setq _target_joint_velocity __target_joint_velocity)
   (setq _target_joint_acceleration __target_joint_acceleration)
   (setq _target_motor_torque __target_motor_torque)
   (setq _target_tcp_position __target_tcp_position)
   (setq _target_tcp_velocity __target_tcp_velocity)
   (setq _jacobian_matrix __jacobian_matrix)
   (setq _gravity_torque __gravity_torque)
   (setq _coriolis_matrix __coriolis_matrix)
   (setq _mass_matrix __mass_matrix)
   (setq _solution_space (round __solution_space))
   (setq _singularity (float __singularity))
   (setq _operation_speed_rate (float __operation_speed_rate))
   (setq _joint_temperature __joint_temperature)
   (setq _controller_digital_input (round __controller_digital_input))
   (setq _controller_digital_output (round __controller_digital_output))
   (setq _controller_analog_input_type __controller_analog_input_type)
   (setq _controller_analog_input __controller_analog_input)
   (setq _controller_analog_output_type __controller_analog_output_type)
   (setq _controller_analog_output __controller_analog_output)
   (setq _flange_digital_input (round __flange_digital_input))
   (setq _flange_digital_output (round __flange_digital_output))
   (setq _flange_analog_input __flange_analog_input)
   (setq _external_encoder_strobe_count __external_encoder_strobe_count)
   (setq _external_encoder_count __external_encoder_count)
   (setq _goal_joint_position __goal_joint_position)
   (setq _goal_tcp_position __goal_tcp_position)
   (setq _robot_mode (round __robot_mode))
   (setq _robot_state (round __robot_state))
   (setq _control_mode (round __control_mode))
   (setq _reserved __reserved)
   self)
  (:time_stamp
   (&optional __time_stamp)
   (if __time_stamp (setq _time_stamp __time_stamp)) _time_stamp)
  (:actual_joint_position
   (&optional __actual_joint_position)
   (if __actual_joint_position (setq _actual_joint_position __actual_joint_position)) _actual_joint_position)
  (:actual_joint_position_abs
   (&optional __actual_joint_position_abs)
   (if __actual_joint_position_abs (setq _actual_joint_position_abs __actual_joint_position_abs)) _actual_joint_position_abs)
  (:actual_joint_velocity
   (&optional __actual_joint_velocity)
   (if __actual_joint_velocity (setq _actual_joint_velocity __actual_joint_velocity)) _actual_joint_velocity)
  (:actual_joint_velocity_abs
   (&optional __actual_joint_velocity_abs)
   (if __actual_joint_velocity_abs (setq _actual_joint_velocity_abs __actual_joint_velocity_abs)) _actual_joint_velocity_abs)
  (:actual_tcp_position
   (&optional __actual_tcp_position)
   (if __actual_tcp_position (setq _actual_tcp_position __actual_tcp_position)) _actual_tcp_position)
  (:actual_tcp_velocity
   (&optional __actual_tcp_velocity)
   (if __actual_tcp_velocity (setq _actual_tcp_velocity __actual_tcp_velocity)) _actual_tcp_velocity)
  (:actual_flange_position
   (&optional __actual_flange_position)
   (if __actual_flange_position (setq _actual_flange_position __actual_flange_position)) _actual_flange_position)
  (:actual_flange_velocity
   (&optional __actual_flange_velocity)
   (if __actual_flange_velocity (setq _actual_flange_velocity __actual_flange_velocity)) _actual_flange_velocity)
  (:actual_motor_torque
   (&optional __actual_motor_torque)
   (if __actual_motor_torque (setq _actual_motor_torque __actual_motor_torque)) _actual_motor_torque)
  (:actual_joint_torque
   (&optional __actual_joint_torque)
   (if __actual_joint_torque (setq _actual_joint_torque __actual_joint_torque)) _actual_joint_torque)
  (:raw_joint_torque
   (&optional __raw_joint_torque)
   (if __raw_joint_torque (setq _raw_joint_torque __raw_joint_torque)) _raw_joint_torque)
  (:raw_force_torque
   (&optional __raw_force_torque)
   (if __raw_force_torque (setq _raw_force_torque __raw_force_torque)) _raw_force_torque)
  (:external_joint_torque
   (&optional __external_joint_torque)
   (if __external_joint_torque (setq _external_joint_torque __external_joint_torque)) _external_joint_torque)
  (:external_tcp_force
   (&optional __external_tcp_force)
   (if __external_tcp_force (setq _external_tcp_force __external_tcp_force)) _external_tcp_force)
  (:target_joint_position
   (&optional __target_joint_position)
   (if __target_joint_position (setq _target_joint_position __target_joint_position)) _target_joint_position)
  (:target_joint_velocity
   (&optional __target_joint_velocity)
   (if __target_joint_velocity (setq _target_joint_velocity __target_joint_velocity)) _target_joint_velocity)
  (:target_joint_acceleration
   (&optional __target_joint_acceleration)
   (if __target_joint_acceleration (setq _target_joint_acceleration __target_joint_acceleration)) _target_joint_acceleration)
  (:target_motor_torque
   (&optional __target_motor_torque)
   (if __target_motor_torque (setq _target_motor_torque __target_motor_torque)) _target_motor_torque)
  (:target_tcp_position
   (&optional __target_tcp_position)
   (if __target_tcp_position (setq _target_tcp_position __target_tcp_position)) _target_tcp_position)
  (:target_tcp_velocity
   (&optional __target_tcp_velocity)
   (if __target_tcp_velocity (setq _target_tcp_velocity __target_tcp_velocity)) _target_tcp_velocity)
  (:jacobian_matrix
   (&rest __jacobian_matrix)
   (if (keywordp (car __jacobian_matrix))
       (send* _jacobian_matrix __jacobian_matrix)
     (progn
       (if __jacobian_matrix (setq _jacobian_matrix (car __jacobian_matrix)))
       _jacobian_matrix)))
  (:gravity_torque
   (&optional __gravity_torque)
   (if __gravity_torque (setq _gravity_torque __gravity_torque)) _gravity_torque)
  (:coriolis_matrix
   (&rest __coriolis_matrix)
   (if (keywordp (car __coriolis_matrix))
       (send* _coriolis_matrix __coriolis_matrix)
     (progn
       (if __coriolis_matrix (setq _coriolis_matrix (car __coriolis_matrix)))
       _coriolis_matrix)))
  (:mass_matrix
   (&rest __mass_matrix)
   (if (keywordp (car __mass_matrix))
       (send* _mass_matrix __mass_matrix)
     (progn
       (if __mass_matrix (setq _mass_matrix (car __mass_matrix)))
       _mass_matrix)))
  (:solution_space
   (&optional __solution_space)
   (if __solution_space (setq _solution_space __solution_space)) _solution_space)
  (:singularity
   (&optional __singularity)
   (if __singularity (setq _singularity __singularity)) _singularity)
  (:operation_speed_rate
   (&optional __operation_speed_rate)
   (if __operation_speed_rate (setq _operation_speed_rate __operation_speed_rate)) _operation_speed_rate)
  (:joint_temperature
   (&optional __joint_temperature)
   (if __joint_temperature (setq _joint_temperature __joint_temperature)) _joint_temperature)
  (:controller_digital_input
   (&optional __controller_digital_input)
   (if __controller_digital_input (setq _controller_digital_input __controller_digital_input)) _controller_digital_input)
  (:controller_digital_output
   (&optional __controller_digital_output)
   (if __controller_digital_output (setq _controller_digital_output __controller_digital_output)) _controller_digital_output)
  (:controller_analog_input_type
   (&optional __controller_analog_input_type)
   (if __controller_analog_input_type (setq _controller_analog_input_type __controller_analog_input_type)) _controller_analog_input_type)
  (:controller_analog_input
   (&optional __controller_analog_input)
   (if __controller_analog_input (setq _controller_analog_input __controller_analog_input)) _controller_analog_input)
  (:controller_analog_output_type
   (&optional __controller_analog_output_type)
   (if __controller_analog_output_type (setq _controller_analog_output_type __controller_analog_output_type)) _controller_analog_output_type)
  (:controller_analog_output
   (&optional __controller_analog_output)
   (if __controller_analog_output (setq _controller_analog_output __controller_analog_output)) _controller_analog_output)
  (:flange_digital_input
   (&optional __flange_digital_input)
   (if __flange_digital_input (setq _flange_digital_input __flange_digital_input)) _flange_digital_input)
  (:flange_digital_output
   (&optional __flange_digital_output)
   (if __flange_digital_output (setq _flange_digital_output __flange_digital_output)) _flange_digital_output)
  (:flange_analog_input
   (&optional __flange_analog_input)
   (if __flange_analog_input (setq _flange_analog_input __flange_analog_input)) _flange_analog_input)
  (:external_encoder_strobe_count
   (&optional __external_encoder_strobe_count)
   (if __external_encoder_strobe_count (setq _external_encoder_strobe_count __external_encoder_strobe_count)) _external_encoder_strobe_count)
  (:external_encoder_count
   (&optional __external_encoder_count)
   (if __external_encoder_count (setq _external_encoder_count __external_encoder_count)) _external_encoder_count)
  (:goal_joint_position
   (&optional __goal_joint_position)
   (if __goal_joint_position (setq _goal_joint_position __goal_joint_position)) _goal_joint_position)
  (:goal_tcp_position
   (&optional __goal_tcp_position)
   (if __goal_tcp_position (setq _goal_tcp_position __goal_tcp_position)) _goal_tcp_position)
  (:robot_mode
   (&optional __robot_mode)
   (if __robot_mode (setq _robot_mode __robot_mode)) _robot_mode)
  (:robot_state
   (&optional __robot_state)
   (if __robot_state (setq _robot_state __robot_state)) _robot_state)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:reserved
   (&optional __reserved)
   (if __reserved (setq _reserved __reserved)) _reserved)
  (:serialization-length
   ()
   (+
    ;; float64 _time_stamp
    8
    ;; float64[6] _actual_joint_position
    (* 8    6)
    ;; float64[6] _actual_joint_position_abs
    (* 8    6)
    ;; float64[6] _actual_joint_velocity
    (* 8    6)
    ;; float64[6] _actual_joint_velocity_abs
    (* 8    6)
    ;; float64[6] _actual_tcp_position
    (* 8    6)
    ;; float64[6] _actual_tcp_velocity
    (* 8    6)
    ;; float64[6] _actual_flange_position
    (* 8    6)
    ;; float64[6] _actual_flange_velocity
    (* 8    6)
    ;; float64[6] _actual_motor_torque
    (* 8    6)
    ;; float64[6] _actual_joint_torque
    (* 8    6)
    ;; float64[6] _raw_joint_torque
    (* 8    6)
    ;; float64[6] _raw_force_torque
    (* 8    6)
    ;; float64[6] _external_joint_torque
    (* 8    6)
    ;; float64[6] _external_tcp_force
    (* 8    6)
    ;; float64[6] _target_joint_position
    (* 8    6)
    ;; float64[6] _target_joint_velocity
    (* 8    6)
    ;; float64[6] _target_joint_acceleration
    (* 8    6)
    ;; float64[6] _target_motor_torque
    (* 8    6)
    ;; float64[6] _target_tcp_position
    (* 8    6)
    ;; float64[6] _target_tcp_velocity
    (* 8    6)
    ;; std_msgs/Float64MultiArray[] _jacobian_matrix
    (apply #'+ (send-all _jacobian_matrix :serialization-length)) 4
    ;; float64[6] _gravity_torque
    (* 8    6)
    ;; std_msgs/Float64MultiArray[] _coriolis_matrix
    (apply #'+ (send-all _coriolis_matrix :serialization-length)) 4
    ;; std_msgs/Float64MultiArray[] _mass_matrix
    (apply #'+ (send-all _mass_matrix :serialization-length)) 4
    ;; uint16 _solution_space
    2
    ;; float64 _singularity
    8
    ;; float64 _operation_speed_rate
    8
    ;; float64[6] _joint_temperature
    (* 8    6)
    ;; uint16 _controller_digital_input
    2
    ;; uint16 _controller_digital_output
    2
    ;; uint8[2] _controller_analog_input_type
    (* 1    2)
    ;; float64[2] _controller_analog_input
    (* 8    2)
    ;; uint8[2] _controller_analog_output_type
    (* 1    2)
    ;; float64[2] _controller_analog_output
    (* 8    2)
    ;; uint8 _flange_digital_input
    1
    ;; uint8 _flange_digital_output
    1
    ;; float64[4] _flange_analog_input
    (* 8    4)
    ;; uint8[2] _external_encoder_strobe_count
    (* 1    2)
    ;; uint16[2] _external_encoder_count
    (* 2    2)
    ;; float64[6] _goal_joint_position
    (* 8    6)
    ;; float64[6] _goal_tcp_position
    (* 8    6)
    ;; uint8 _robot_mode
    1
    ;; uint8 _robot_state
    1
    ;; uint16 _control_mode
    2
    ;; uint8[256] _reserved
    (* 1    256)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _time_stamp
       (sys::poke _time_stamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[6] _actual_joint_position
     (dotimes (i 6)
       (sys::poke (elt _actual_joint_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _actual_joint_position_abs
     (dotimes (i 6)
       (sys::poke (elt _actual_joint_position_abs i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _actual_joint_velocity
     (dotimes (i 6)
       (sys::poke (elt _actual_joint_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _actual_joint_velocity_abs
     (dotimes (i 6)
       (sys::poke (elt _actual_joint_velocity_abs i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _actual_tcp_position
     (dotimes (i 6)
       (sys::poke (elt _actual_tcp_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _actual_tcp_velocity
     (dotimes (i 6)
       (sys::poke (elt _actual_tcp_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _actual_flange_position
     (dotimes (i 6)
       (sys::poke (elt _actual_flange_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _actual_flange_velocity
     (dotimes (i 6)
       (sys::poke (elt _actual_flange_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _actual_motor_torque
     (dotimes (i 6)
       (sys::poke (elt _actual_motor_torque i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _actual_joint_torque
     (dotimes (i 6)
       (sys::poke (elt _actual_joint_torque i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _raw_joint_torque
     (dotimes (i 6)
       (sys::poke (elt _raw_joint_torque i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _raw_force_torque
     (dotimes (i 6)
       (sys::poke (elt _raw_force_torque i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _external_joint_torque
     (dotimes (i 6)
       (sys::poke (elt _external_joint_torque i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _external_tcp_force
     (dotimes (i 6)
       (sys::poke (elt _external_tcp_force i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _target_joint_position
     (dotimes (i 6)
       (sys::poke (elt _target_joint_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _target_joint_velocity
     (dotimes (i 6)
       (sys::poke (elt _target_joint_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _target_joint_acceleration
     (dotimes (i 6)
       (sys::poke (elt _target_joint_acceleration i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _target_motor_torque
     (dotimes (i 6)
       (sys::poke (elt _target_motor_torque i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _target_tcp_position
     (dotimes (i 6)
       (sys::poke (elt _target_tcp_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _target_tcp_velocity
     (dotimes (i 6)
       (sys::poke (elt _target_tcp_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; std_msgs/Float64MultiArray[] _jacobian_matrix
     (write-long (length _jacobian_matrix) s)
     (dolist (elem _jacobian_matrix)
       (send elem :serialize s)
       )
     ;; float64[6] _gravity_torque
     (dotimes (i 6)
       (sys::poke (elt _gravity_torque i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; std_msgs/Float64MultiArray[] _coriolis_matrix
     (write-long (length _coriolis_matrix) s)
     (dolist (elem _coriolis_matrix)
       (send elem :serialize s)
       )
     ;; std_msgs/Float64MultiArray[] _mass_matrix
     (write-long (length _mass_matrix) s)
     (dolist (elem _mass_matrix)
       (send elem :serialize s)
       )
     ;; uint16 _solution_space
       (write-word _solution_space s)
     ;; float64 _singularity
       (sys::poke _singularity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _operation_speed_rate
       (sys::poke _operation_speed_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[6] _joint_temperature
     (dotimes (i 6)
       (sys::poke (elt _joint_temperature i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; uint16 _controller_digital_input
       (write-word _controller_digital_input s)
     ;; uint16 _controller_digital_output
       (write-word _controller_digital_output s)
     ;; uint8[2] _controller_analog_input_type
     (princ _controller_analog_input_type s)
     ;; float64[2] _controller_analog_input
     (dotimes (i 2)
       (sys::poke (elt _controller_analog_input i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; uint8[2] _controller_analog_output_type
     (princ _controller_analog_output_type s)
     ;; float64[2] _controller_analog_output
     (dotimes (i 2)
       (sys::poke (elt _controller_analog_output i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; uint8 _flange_digital_input
       (write-byte _flange_digital_input s)
     ;; uint8 _flange_digital_output
       (write-byte _flange_digital_output s)
     ;; float64[4] _flange_analog_input
     (dotimes (i 4)
       (sys::poke (elt _flange_analog_input i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; uint8[2] _external_encoder_strobe_count
     (princ _external_encoder_strobe_count s)
     ;; uint16[2] _external_encoder_count
     (dotimes (i 2)
       (write-word (elt _external_encoder_count i) s)
       )
     ;; float64[6] _goal_joint_position
     (dotimes (i 6)
       (sys::poke (elt _goal_joint_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _goal_tcp_position
     (dotimes (i 6)
       (sys::poke (elt _goal_tcp_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; uint8 _robot_mode
       (write-byte _robot_mode s)
     ;; uint8 _robot_state
       (write-byte _robot_state s)
     ;; uint16 _control_mode
       (write-word _control_mode s)
     ;; uint8[256] _reserved
     (princ _reserved s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _time_stamp
     (setq _time_stamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[6] _actual_joint_position
   (dotimes (i (length _actual_joint_position))
     (setf (elt _actual_joint_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _actual_joint_position_abs
   (dotimes (i (length _actual_joint_position_abs))
     (setf (elt _actual_joint_position_abs i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _actual_joint_velocity
   (dotimes (i (length _actual_joint_velocity))
     (setf (elt _actual_joint_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _actual_joint_velocity_abs
   (dotimes (i (length _actual_joint_velocity_abs))
     (setf (elt _actual_joint_velocity_abs i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _actual_tcp_position
   (dotimes (i (length _actual_tcp_position))
     (setf (elt _actual_tcp_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _actual_tcp_velocity
   (dotimes (i (length _actual_tcp_velocity))
     (setf (elt _actual_tcp_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _actual_flange_position
   (dotimes (i (length _actual_flange_position))
     (setf (elt _actual_flange_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _actual_flange_velocity
   (dotimes (i (length _actual_flange_velocity))
     (setf (elt _actual_flange_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _actual_motor_torque
   (dotimes (i (length _actual_motor_torque))
     (setf (elt _actual_motor_torque i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _actual_joint_torque
   (dotimes (i (length _actual_joint_torque))
     (setf (elt _actual_joint_torque i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _raw_joint_torque
   (dotimes (i (length _raw_joint_torque))
     (setf (elt _raw_joint_torque i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _raw_force_torque
   (dotimes (i (length _raw_force_torque))
     (setf (elt _raw_force_torque i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _external_joint_torque
   (dotimes (i (length _external_joint_torque))
     (setf (elt _external_joint_torque i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _external_tcp_force
   (dotimes (i (length _external_tcp_force))
     (setf (elt _external_tcp_force i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _target_joint_position
   (dotimes (i (length _target_joint_position))
     (setf (elt _target_joint_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _target_joint_velocity
   (dotimes (i (length _target_joint_velocity))
     (setf (elt _target_joint_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _target_joint_acceleration
   (dotimes (i (length _target_joint_acceleration))
     (setf (elt _target_joint_acceleration i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _target_motor_torque
   (dotimes (i (length _target_motor_torque))
     (setf (elt _target_motor_torque i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _target_tcp_position
   (dotimes (i (length _target_tcp_position))
     (setf (elt _target_tcp_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _target_tcp_velocity
   (dotimes (i (length _target_tcp_velocity))
     (setf (elt _target_tcp_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; std_msgs/Float64MultiArray[] _jacobian_matrix
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _jacobian_matrix (let (r) (dotimes (i n) (push (instance std_msgs::Float64MultiArray :init) r)) r))
     (dolist (elem- _jacobian_matrix)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[6] _gravity_torque
   (dotimes (i (length _gravity_torque))
     (setf (elt _gravity_torque i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; std_msgs/Float64MultiArray[] _coriolis_matrix
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _coriolis_matrix (let (r) (dotimes (i n) (push (instance std_msgs::Float64MultiArray :init) r)) r))
     (dolist (elem- _coriolis_matrix)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/Float64MultiArray[] _mass_matrix
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _mass_matrix (let (r) (dotimes (i n) (push (instance std_msgs::Float64MultiArray :init) r)) r))
     (dolist (elem- _mass_matrix)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint16 _solution_space
     (setq _solution_space (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _singularity
     (setq _singularity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _operation_speed_rate
     (setq _operation_speed_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[6] _joint_temperature
   (dotimes (i (length _joint_temperature))
     (setf (elt _joint_temperature i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; uint16 _controller_digital_input
     (setq _controller_digital_input (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _controller_digital_output
     (setq _controller_digital_output (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8[2] _controller_analog_input_type
   (setq _controller_analog_input_type (make-array 2 :element-type :char))
   (replace _controller_analog_input_type buf :start2 ptr-) (incf ptr- 2)
   ;; float64[2] _controller_analog_input
   (dotimes (i (length _controller_analog_input))
     (setf (elt _controller_analog_input i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; uint8[2] _controller_analog_output_type
   (setq _controller_analog_output_type (make-array 2 :element-type :char))
   (replace _controller_analog_output_type buf :start2 ptr-) (incf ptr- 2)
   ;; float64[2] _controller_analog_output
   (dotimes (i (length _controller_analog_output))
     (setf (elt _controller_analog_output i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; uint8 _flange_digital_input
     (setq _flange_digital_input (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _flange_digital_output
     (setq _flange_digital_output (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64[4] _flange_analog_input
   (dotimes (i (length _flange_analog_input))
     (setf (elt _flange_analog_input i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; uint8[2] _external_encoder_strobe_count
   (setq _external_encoder_strobe_count (make-array 2 :element-type :char))
   (replace _external_encoder_strobe_count buf :start2 ptr-) (incf ptr- 2)
   ;; uint16[2] _external_encoder_count
   (dotimes (i (length _external_encoder_count))
     (setf (elt _external_encoder_count i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; float64[6] _goal_joint_position
   (dotimes (i (length _goal_joint_position))
     (setf (elt _goal_joint_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _goal_tcp_position
   (dotimes (i (length _goal_tcp_position))
     (setf (elt _goal_tcp_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; uint8 _robot_mode
     (setq _robot_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _robot_state
     (setq _robot_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _control_mode
     (setq _control_mode (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8[256] _reserved
   (setq _reserved (make-array 256 :element-type :char))
   (replace _reserved buf :start2 ptr-) (incf ptr- 256)
   ;;
   self)
  )

(setf (get dsr_msgs::RobotStateRT :md5sum-) "3fc4bdbba50c12ddf9126a58ad825fc4")
(setf (get dsr_msgs::RobotStateRT :datatype-) "dsr_msgs/RobotStateRT")
(setf (get dsr_msgs::RobotStateRT :definition-)
      "# timestamp at the data of data acquisition
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
uint32 stride  # stride of given dimension
")



(provide :dsr_msgs/RobotStateRT "3fc4bdbba50c12ddf9126a58ad825fc4")


