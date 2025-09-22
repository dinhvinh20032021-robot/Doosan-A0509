;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::RobotState)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'RobotState (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::ROBOTSTATE")
  (make-package "DSR_MSGS::ROBOTSTATE"))

(in-package "ROS")
;;//! \htmlinclude RobotState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dsr_msgs::RobotState
  :super ros::object
  :slots (_robot_state _robot_state_str _actual_mode _actual_space _current_posj _current_velj _joint_abs _joint_err _target_posj _target_velj _current_posx _current_tool_posx _current_velx _task_err _target_velx _target_posx _dynamic_tor _actual_jts _actual_ejt _actual_ett _actual_bk _actual_mc _actual_mt _solution_space _sync_time _actual_bt _rotation_matrix _ctrlbox_digital_input _ctrlbox_digital_output _flange_digital_input _flange_digital_output _modbus_state _access_control _homming_completed _tp_initialized _mastering_need _drl_stopped _disconnected _fActualW2B _fCurrentPosW _fCurrentVelW _fWorldETT _fTargetPosW _fTargetVelW _fRotationMatrixWorld _iActualUCN _iParent _fCurrentPosU _fCurrentVelU _fUserETT _fTargetPosU _fTargetVelU _fRotationMatrixUser _fActualAI _bActualSW _bActualSI _iActualAT _fTargetAO _iTargetAT _bActualES _iActualED _bActualER ))

(defmethod dsr_msgs::RobotState
  (:init
   (&key
    ((:robot_state __robot_state) 0)
    ((:robot_state_str __robot_state_str) "")
    ((:actual_mode __actual_mode) 0)
    ((:actual_space __actual_space) 0)
    ((:current_posj __current_posj) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:current_velj __current_velj) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:joint_abs __joint_abs) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:joint_err __joint_err) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:target_posj __target_posj) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:target_velj __target_velj) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:current_posx __current_posx) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:current_tool_posx __current_tool_posx) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:current_velx __current_velx) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:task_err __task_err) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:target_velx __target_velx) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:target_posx __target_posx) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:dynamic_tor __dynamic_tor) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:actual_jts __actual_jts) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:actual_ejt __actual_ejt) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:actual_ett __actual_ett) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:actual_bk __actual_bk) (make-array 6 :initial-element 0 :element-type :integer))
    ((:actual_mc __actual_mc) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:actual_mt __actual_mt) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:solution_space __solution_space) 0)
    ((:sync_time __sync_time) 0.0)
    ((:actual_bt __actual_bt) (make-array 5 :initial-element 0 :element-type :integer))
    ((:rotation_matrix __rotation_matrix) ())
    ((:ctrlbox_digital_input __ctrlbox_digital_input) (make-array 16 :initial-element 0 :element-type :integer))
    ((:ctrlbox_digital_output __ctrlbox_digital_output) (make-array 16 :initial-element 0 :element-type :integer))
    ((:flange_digital_input __flange_digital_input) (make-array 6 :initial-element 0 :element-type :integer))
    ((:flange_digital_output __flange_digital_output) (make-array 6 :initial-element 0 :element-type :integer))
    ((:modbus_state __modbus_state) ())
    ((:access_control __access_control) 0)
    ((:homming_completed __homming_completed) nil)
    ((:tp_initialized __tp_initialized) nil)
    ((:mastering_need __mastering_need) 0)
    ((:drl_stopped __drl_stopped) nil)
    ((:disconnected __disconnected) nil)
    ((:fActualW2B __fActualW2B) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:fCurrentPosW __fCurrentPosW) ())
    ((:fCurrentVelW __fCurrentVelW) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:fWorldETT __fWorldETT) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:fTargetPosW __fTargetPosW) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:fTargetVelW __fTargetVelW) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:fRotationMatrixWorld __fRotationMatrixWorld) ())
    ((:iActualUCN __iActualUCN) 0)
    ((:iParent __iParent) 0)
    ((:fCurrentPosU __fCurrentPosU) ())
    ((:fCurrentVelU __fCurrentVelU) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:fUserETT __fUserETT) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:fTargetPosU __fTargetPosU) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:fTargetVelU __fTargetVelU) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:fRotationMatrixUser __fRotationMatrixUser) ())
    ((:fActualAI __fActualAI) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:bActualSW __bActualSW) (let (r) (dotimes (i 3) (push nil r)) r))
    ((:bActualSI __bActualSI) (let (r) (dotimes (i 2) (push nil r)) r))
    ((:iActualAT __iActualAT) (make-array 2 :initial-element 0 :element-type :integer))
    ((:fTargetAO __fTargetAO) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:iTargetAT __iTargetAT) (make-array 2 :initial-element 0 :element-type :integer))
    ((:bActualES __bActualES) (let (r) (dotimes (i 2) (push nil r)) r))
    ((:iActualED __iActualED) (make-array 2 :initial-element 0 :element-type :integer))
    ((:bActualER __bActualER) (let (r) (dotimes (i 2) (push nil r)) r))
    )
   (send-super :init)
   (setq _robot_state (round __robot_state))
   (setq _robot_state_str (string __robot_state_str))
   (setq _actual_mode (round __actual_mode))
   (setq _actual_space (round __actual_space))
   (setq _current_posj __current_posj)
   (setq _current_velj __current_velj)
   (setq _joint_abs __joint_abs)
   (setq _joint_err __joint_err)
   (setq _target_posj __target_posj)
   (setq _target_velj __target_velj)
   (setq _current_posx __current_posx)
   (setq _current_tool_posx __current_tool_posx)
   (setq _current_velx __current_velx)
   (setq _task_err __task_err)
   (setq _target_velx __target_velx)
   (setq _target_posx __target_posx)
   (setq _dynamic_tor __dynamic_tor)
   (setq _actual_jts __actual_jts)
   (setq _actual_ejt __actual_ejt)
   (setq _actual_ett __actual_ett)
   (setq _actual_bk __actual_bk)
   (setq _actual_mc __actual_mc)
   (setq _actual_mt __actual_mt)
   (setq _solution_space (round __solution_space))
   (setq _sync_time (float __sync_time))
   (setq _actual_bt __actual_bt)
   (setq _rotation_matrix __rotation_matrix)
   (setq _ctrlbox_digital_input __ctrlbox_digital_input)
   (setq _ctrlbox_digital_output __ctrlbox_digital_output)
   (setq _flange_digital_input __flange_digital_input)
   (setq _flange_digital_output __flange_digital_output)
   (setq _modbus_state __modbus_state)
   (setq _access_control (round __access_control))
   (setq _homming_completed __homming_completed)
   (setq _tp_initialized __tp_initialized)
   (setq _mastering_need (round __mastering_need))
   (setq _drl_stopped __drl_stopped)
   (setq _disconnected __disconnected)
   (setq _fActualW2B __fActualW2B)
   (setq _fCurrentPosW __fCurrentPosW)
   (setq _fCurrentVelW __fCurrentVelW)
   (setq _fWorldETT __fWorldETT)
   (setq _fTargetPosW __fTargetPosW)
   (setq _fTargetVelW __fTargetVelW)
   (setq _fRotationMatrixWorld __fRotationMatrixWorld)
   (setq _iActualUCN (round __iActualUCN))
   (setq _iParent (round __iParent))
   (setq _fCurrentPosU __fCurrentPosU)
   (setq _fCurrentVelU __fCurrentVelU)
   (setq _fUserETT __fUserETT)
   (setq _fTargetPosU __fTargetPosU)
   (setq _fTargetVelU __fTargetVelU)
   (setq _fRotationMatrixUser __fRotationMatrixUser)
   (setq _fActualAI __fActualAI)
   (setq _bActualSW __bActualSW)
   (setq _bActualSI __bActualSI)
   (setq _iActualAT __iActualAT)
   (setq _fTargetAO __fTargetAO)
   (setq _iTargetAT __iTargetAT)
   (setq _bActualES __bActualES)
   (setq _iActualED __iActualED)
   (setq _bActualER __bActualER)
   self)
  (:robot_state
   (&optional __robot_state)
   (if __robot_state (setq _robot_state __robot_state)) _robot_state)
  (:robot_state_str
   (&optional __robot_state_str)
   (if __robot_state_str (setq _robot_state_str __robot_state_str)) _robot_state_str)
  (:actual_mode
   (&optional __actual_mode)
   (if __actual_mode (setq _actual_mode __actual_mode)) _actual_mode)
  (:actual_space
   (&optional __actual_space)
   (if __actual_space (setq _actual_space __actual_space)) _actual_space)
  (:current_posj
   (&optional __current_posj)
   (if __current_posj (setq _current_posj __current_posj)) _current_posj)
  (:current_velj
   (&optional __current_velj)
   (if __current_velj (setq _current_velj __current_velj)) _current_velj)
  (:joint_abs
   (&optional __joint_abs)
   (if __joint_abs (setq _joint_abs __joint_abs)) _joint_abs)
  (:joint_err
   (&optional __joint_err)
   (if __joint_err (setq _joint_err __joint_err)) _joint_err)
  (:target_posj
   (&optional __target_posj)
   (if __target_posj (setq _target_posj __target_posj)) _target_posj)
  (:target_velj
   (&optional __target_velj)
   (if __target_velj (setq _target_velj __target_velj)) _target_velj)
  (:current_posx
   (&optional __current_posx)
   (if __current_posx (setq _current_posx __current_posx)) _current_posx)
  (:current_tool_posx
   (&optional __current_tool_posx)
   (if __current_tool_posx (setq _current_tool_posx __current_tool_posx)) _current_tool_posx)
  (:current_velx
   (&optional __current_velx)
   (if __current_velx (setq _current_velx __current_velx)) _current_velx)
  (:task_err
   (&optional __task_err)
   (if __task_err (setq _task_err __task_err)) _task_err)
  (:target_velx
   (&optional __target_velx)
   (if __target_velx (setq _target_velx __target_velx)) _target_velx)
  (:target_posx
   (&optional __target_posx)
   (if __target_posx (setq _target_posx __target_posx)) _target_posx)
  (:dynamic_tor
   (&optional __dynamic_tor)
   (if __dynamic_tor (setq _dynamic_tor __dynamic_tor)) _dynamic_tor)
  (:actual_jts
   (&optional __actual_jts)
   (if __actual_jts (setq _actual_jts __actual_jts)) _actual_jts)
  (:actual_ejt
   (&optional __actual_ejt)
   (if __actual_ejt (setq _actual_ejt __actual_ejt)) _actual_ejt)
  (:actual_ett
   (&optional __actual_ett)
   (if __actual_ett (setq _actual_ett __actual_ett)) _actual_ett)
  (:actual_bk
   (&optional __actual_bk)
   (if __actual_bk (setq _actual_bk __actual_bk)) _actual_bk)
  (:actual_mc
   (&optional __actual_mc)
   (if __actual_mc (setq _actual_mc __actual_mc)) _actual_mc)
  (:actual_mt
   (&optional __actual_mt)
   (if __actual_mt (setq _actual_mt __actual_mt)) _actual_mt)
  (:solution_space
   (&optional __solution_space)
   (if __solution_space (setq _solution_space __solution_space)) _solution_space)
  (:sync_time
   (&optional __sync_time)
   (if __sync_time (setq _sync_time __sync_time)) _sync_time)
  (:actual_bt
   (&optional __actual_bt)
   (if __actual_bt (setq _actual_bt __actual_bt)) _actual_bt)
  (:rotation_matrix
   (&rest __rotation_matrix)
   (if (keywordp (car __rotation_matrix))
       (send* _rotation_matrix __rotation_matrix)
     (progn
       (if __rotation_matrix (setq _rotation_matrix (car __rotation_matrix)))
       _rotation_matrix)))
  (:ctrlbox_digital_input
   (&optional __ctrlbox_digital_input)
   (if __ctrlbox_digital_input (setq _ctrlbox_digital_input __ctrlbox_digital_input)) _ctrlbox_digital_input)
  (:ctrlbox_digital_output
   (&optional __ctrlbox_digital_output)
   (if __ctrlbox_digital_output (setq _ctrlbox_digital_output __ctrlbox_digital_output)) _ctrlbox_digital_output)
  (:flange_digital_input
   (&optional __flange_digital_input)
   (if __flange_digital_input (setq _flange_digital_input __flange_digital_input)) _flange_digital_input)
  (:flange_digital_output
   (&optional __flange_digital_output)
   (if __flange_digital_output (setq _flange_digital_output __flange_digital_output)) _flange_digital_output)
  (:modbus_state
   (&rest __modbus_state)
   (if (keywordp (car __modbus_state))
       (send* _modbus_state __modbus_state)
     (progn
       (if __modbus_state (setq _modbus_state (car __modbus_state)))
       _modbus_state)))
  (:access_control
   (&optional __access_control)
   (if __access_control (setq _access_control __access_control)) _access_control)
  (:homming_completed
   (&optional (__homming_completed :null))
   (if (not (eq __homming_completed :null)) (setq _homming_completed __homming_completed)) _homming_completed)
  (:tp_initialized
   (&optional (__tp_initialized :null))
   (if (not (eq __tp_initialized :null)) (setq _tp_initialized __tp_initialized)) _tp_initialized)
  (:mastering_need
   (&optional __mastering_need)
   (if __mastering_need (setq _mastering_need __mastering_need)) _mastering_need)
  (:drl_stopped
   (&optional (__drl_stopped :null))
   (if (not (eq __drl_stopped :null)) (setq _drl_stopped __drl_stopped)) _drl_stopped)
  (:disconnected
   (&optional (__disconnected :null))
   (if (not (eq __disconnected :null)) (setq _disconnected __disconnected)) _disconnected)
  (:fActualW2B
   (&optional __fActualW2B)
   (if __fActualW2B (setq _fActualW2B __fActualW2B)) _fActualW2B)
  (:fCurrentPosW
   (&rest __fCurrentPosW)
   (if (keywordp (car __fCurrentPosW))
       (send* _fCurrentPosW __fCurrentPosW)
     (progn
       (if __fCurrentPosW (setq _fCurrentPosW (car __fCurrentPosW)))
       _fCurrentPosW)))
  (:fCurrentVelW
   (&optional __fCurrentVelW)
   (if __fCurrentVelW (setq _fCurrentVelW __fCurrentVelW)) _fCurrentVelW)
  (:fWorldETT
   (&optional __fWorldETT)
   (if __fWorldETT (setq _fWorldETT __fWorldETT)) _fWorldETT)
  (:fTargetPosW
   (&optional __fTargetPosW)
   (if __fTargetPosW (setq _fTargetPosW __fTargetPosW)) _fTargetPosW)
  (:fTargetVelW
   (&optional __fTargetVelW)
   (if __fTargetVelW (setq _fTargetVelW __fTargetVelW)) _fTargetVelW)
  (:fRotationMatrixWorld
   (&rest __fRotationMatrixWorld)
   (if (keywordp (car __fRotationMatrixWorld))
       (send* _fRotationMatrixWorld __fRotationMatrixWorld)
     (progn
       (if __fRotationMatrixWorld (setq _fRotationMatrixWorld (car __fRotationMatrixWorld)))
       _fRotationMatrixWorld)))
  (:iActualUCN
   (&optional __iActualUCN)
   (if __iActualUCN (setq _iActualUCN __iActualUCN)) _iActualUCN)
  (:iParent
   (&optional __iParent)
   (if __iParent (setq _iParent __iParent)) _iParent)
  (:fCurrentPosU
   (&rest __fCurrentPosU)
   (if (keywordp (car __fCurrentPosU))
       (send* _fCurrentPosU __fCurrentPosU)
     (progn
       (if __fCurrentPosU (setq _fCurrentPosU (car __fCurrentPosU)))
       _fCurrentPosU)))
  (:fCurrentVelU
   (&optional __fCurrentVelU)
   (if __fCurrentVelU (setq _fCurrentVelU __fCurrentVelU)) _fCurrentVelU)
  (:fUserETT
   (&optional __fUserETT)
   (if __fUserETT (setq _fUserETT __fUserETT)) _fUserETT)
  (:fTargetPosU
   (&optional __fTargetPosU)
   (if __fTargetPosU (setq _fTargetPosU __fTargetPosU)) _fTargetPosU)
  (:fTargetVelU
   (&optional __fTargetVelU)
   (if __fTargetVelU (setq _fTargetVelU __fTargetVelU)) _fTargetVelU)
  (:fRotationMatrixUser
   (&rest __fRotationMatrixUser)
   (if (keywordp (car __fRotationMatrixUser))
       (send* _fRotationMatrixUser __fRotationMatrixUser)
     (progn
       (if __fRotationMatrixUser (setq _fRotationMatrixUser (car __fRotationMatrixUser)))
       _fRotationMatrixUser)))
  (:fActualAI
   (&optional __fActualAI)
   (if __fActualAI (setq _fActualAI __fActualAI)) _fActualAI)
  (:bActualSW
   (&optional __bActualSW)
   (if __bActualSW (setq _bActualSW __bActualSW)) _bActualSW)
  (:bActualSI
   (&optional __bActualSI)
   (if __bActualSI (setq _bActualSI __bActualSI)) _bActualSI)
  (:iActualAT
   (&optional __iActualAT)
   (if __iActualAT (setq _iActualAT __iActualAT)) _iActualAT)
  (:fTargetAO
   (&optional __fTargetAO)
   (if __fTargetAO (setq _fTargetAO __fTargetAO)) _fTargetAO)
  (:iTargetAT
   (&optional __iTargetAT)
   (if __iTargetAT (setq _iTargetAT __iTargetAT)) _iTargetAT)
  (:bActualES
   (&optional __bActualES)
   (if __bActualES (setq _bActualES __bActualES)) _bActualES)
  (:iActualED
   (&optional __iActualED)
   (if __iActualED (setq _iActualED __iActualED)) _iActualED)
  (:bActualER
   (&optional __bActualER)
   (if __bActualER (setq _bActualER __bActualER)) _bActualER)
  (:serialization-length
   ()
   (+
    ;; int32 _robot_state
    4
    ;; string _robot_state_str
    4 (length _robot_state_str)
    ;; int8 _actual_mode
    1
    ;; int8 _actual_space
    1
    ;; float64[6] _current_posj
    (* 8    6)
    ;; float64[6] _current_velj
    (* 8    6)
    ;; float64[6] _joint_abs
    (* 8    6)
    ;; float64[6] _joint_err
    (* 8    6)
    ;; float64[6] _target_posj
    (* 8    6)
    ;; float64[6] _target_velj
    (* 8    6)
    ;; float64[6] _current_posx
    (* 8    6)
    ;; float64[6] _current_tool_posx
    (* 8    6)
    ;; float64[6] _current_velx
    (* 8    6)
    ;; float64[6] _task_err
    (* 8    6)
    ;; float64[6] _target_velx
    (* 8    6)
    ;; float64[6] _target_posx
    (* 8    6)
    ;; float64[6] _dynamic_tor
    (* 8    6)
    ;; float64[6] _actual_jts
    (* 8    6)
    ;; float64[6] _actual_ejt
    (* 8    6)
    ;; float64[6] _actual_ett
    (* 8    6)
    ;; int8[6] _actual_bk
    (* 1    6)
    ;; float64[6] _actual_mc
    (* 8    6)
    ;; float64[6] _actual_mt
    (* 8    6)
    ;; int8 _solution_space
    1
    ;; float64 _sync_time
    8
    ;; int8[5] _actual_bt
    (* 1    5)
    ;; std_msgs/Float64MultiArray[] _rotation_matrix
    (apply #'+ (send-all _rotation_matrix :serialization-length)) 4
    ;; int8[16] _ctrlbox_digital_input
    (* 1    16)
    ;; int8[16] _ctrlbox_digital_output
    (* 1    16)
    ;; int8[6] _flange_digital_input
    (* 1    6)
    ;; int8[6] _flange_digital_output
    (* 1    6)
    ;; dsr_msgs/ModbusState[] _modbus_state
    (apply #'+ (send-all _modbus_state :serialization-length)) 4
    ;; int32 _access_control
    4
    ;; bool _homming_completed
    1
    ;; bool _tp_initialized
    1
    ;; int8 _mastering_need
    1
    ;; bool _drl_stopped
    1
    ;; bool _disconnected
    1
    ;; float64[6] _fActualW2B
    (* 8    6)
    ;; std_msgs/Float64MultiArray[] _fCurrentPosW
    (apply #'+ (send-all _fCurrentPosW :serialization-length)) 4
    ;; float64[6] _fCurrentVelW
    (* 8    6)
    ;; float64[6] _fWorldETT
    (* 8    6)
    ;; float64[6] _fTargetPosW
    (* 8    6)
    ;; float64[6] _fTargetVelW
    (* 8    6)
    ;; std_msgs/Float64MultiArray[] _fRotationMatrixWorld
    (apply #'+ (send-all _fRotationMatrixWorld :serialization-length)) 4
    ;; int8 _iActualUCN
    1
    ;; int8 _iParent
    1
    ;; std_msgs/Float64MultiArray[] _fCurrentPosU
    (apply #'+ (send-all _fCurrentPosU :serialization-length)) 4
    ;; float64[6] _fCurrentVelU
    (* 8    6)
    ;; float64[6] _fUserETT
    (* 8    6)
    ;; float64[6] _fTargetPosU
    (* 8    6)
    ;; float64[6] _fTargetVelU
    (* 8    6)
    ;; std_msgs/Float64MultiArray[] _fRotationMatrixUser
    (apply #'+ (send-all _fRotationMatrixUser :serialization-length)) 4
    ;; float64[6] _fActualAI
    (* 8    6)
    ;; bool[3] _bActualSW
    (* 1    3)
    ;; bool[2] _bActualSI
    (* 1    2)
    ;; int8[2] _iActualAT
    (* 1    2)
    ;; float64[2] _fTargetAO
    (* 8    2)
    ;; int8[2] _iTargetAT
    (* 1    2)
    ;; bool[2] _bActualES
    (* 1    2)
    ;; int8[2] _iActualED
    (* 1    2)
    ;; bool[2] _bActualER
    (* 1    2)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _robot_state
       (write-long _robot_state s)
     ;; string _robot_state_str
       (write-long (length _robot_state_str) s) (princ _robot_state_str s)
     ;; int8 _actual_mode
       (write-byte _actual_mode s)
     ;; int8 _actual_space
       (write-byte _actual_space s)
     ;; float64[6] _current_posj
     (dotimes (i 6)
       (sys::poke (elt _current_posj i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _current_velj
     (dotimes (i 6)
       (sys::poke (elt _current_velj i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _joint_abs
     (dotimes (i 6)
       (sys::poke (elt _joint_abs i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _joint_err
     (dotimes (i 6)
       (sys::poke (elt _joint_err i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _target_posj
     (dotimes (i 6)
       (sys::poke (elt _target_posj i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _target_velj
     (dotimes (i 6)
       (sys::poke (elt _target_velj i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _current_posx
     (dotimes (i 6)
       (sys::poke (elt _current_posx i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _current_tool_posx
     (dotimes (i 6)
       (sys::poke (elt _current_tool_posx i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _current_velx
     (dotimes (i 6)
       (sys::poke (elt _current_velx i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _task_err
     (dotimes (i 6)
       (sys::poke (elt _task_err i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _target_velx
     (dotimes (i 6)
       (sys::poke (elt _target_velx i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _target_posx
     (dotimes (i 6)
       (sys::poke (elt _target_posx i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _dynamic_tor
     (dotimes (i 6)
       (sys::poke (elt _dynamic_tor i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _actual_jts
     (dotimes (i 6)
       (sys::poke (elt _actual_jts i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _actual_ejt
     (dotimes (i 6)
       (sys::poke (elt _actual_ejt i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _actual_ett
     (dotimes (i 6)
       (sys::poke (elt _actual_ett i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8[6] _actual_bk
     (dotimes (i 6)
       (write-byte (elt _actual_bk i) s)
       )
     ;; float64[6] _actual_mc
     (dotimes (i 6)
       (sys::poke (elt _actual_mc i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _actual_mt
     (dotimes (i 6)
       (sys::poke (elt _actual_mt i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8 _solution_space
       (write-byte _solution_space s)
     ;; float64 _sync_time
       (sys::poke _sync_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8[5] _actual_bt
     (dotimes (i 5)
       (write-byte (elt _actual_bt i) s)
       )
     ;; std_msgs/Float64MultiArray[] _rotation_matrix
     (write-long (length _rotation_matrix) s)
     (dolist (elem _rotation_matrix)
       (send elem :serialize s)
       )
     ;; int8[16] _ctrlbox_digital_input
     (dotimes (i 16)
       (write-byte (elt _ctrlbox_digital_input i) s)
       )
     ;; int8[16] _ctrlbox_digital_output
     (dotimes (i 16)
       (write-byte (elt _ctrlbox_digital_output i) s)
       )
     ;; int8[6] _flange_digital_input
     (dotimes (i 6)
       (write-byte (elt _flange_digital_input i) s)
       )
     ;; int8[6] _flange_digital_output
     (dotimes (i 6)
       (write-byte (elt _flange_digital_output i) s)
       )
     ;; dsr_msgs/ModbusState[] _modbus_state
     (write-long (length _modbus_state) s)
     (dolist (elem _modbus_state)
       (send elem :serialize s)
       )
     ;; int32 _access_control
       (write-long _access_control s)
     ;; bool _homming_completed
       (if _homming_completed (write-byte -1 s) (write-byte 0 s))
     ;; bool _tp_initialized
       (if _tp_initialized (write-byte -1 s) (write-byte 0 s))
     ;; int8 _mastering_need
       (write-byte _mastering_need s)
     ;; bool _drl_stopped
       (if _drl_stopped (write-byte -1 s) (write-byte 0 s))
     ;; bool _disconnected
       (if _disconnected (write-byte -1 s) (write-byte 0 s))
     ;; float64[6] _fActualW2B
     (dotimes (i 6)
       (sys::poke (elt _fActualW2B i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; std_msgs/Float64MultiArray[] _fCurrentPosW
     (write-long (length _fCurrentPosW) s)
     (dolist (elem _fCurrentPosW)
       (send elem :serialize s)
       )
     ;; float64[6] _fCurrentVelW
     (dotimes (i 6)
       (sys::poke (elt _fCurrentVelW i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _fWorldETT
     (dotimes (i 6)
       (sys::poke (elt _fWorldETT i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _fTargetPosW
     (dotimes (i 6)
       (sys::poke (elt _fTargetPosW i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _fTargetVelW
     (dotimes (i 6)
       (sys::poke (elt _fTargetVelW i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; std_msgs/Float64MultiArray[] _fRotationMatrixWorld
     (write-long (length _fRotationMatrixWorld) s)
     (dolist (elem _fRotationMatrixWorld)
       (send elem :serialize s)
       )
     ;; int8 _iActualUCN
       (write-byte _iActualUCN s)
     ;; int8 _iParent
       (write-byte _iParent s)
     ;; std_msgs/Float64MultiArray[] _fCurrentPosU
     (write-long (length _fCurrentPosU) s)
     (dolist (elem _fCurrentPosU)
       (send elem :serialize s)
       )
     ;; float64[6] _fCurrentVelU
     (dotimes (i 6)
       (sys::poke (elt _fCurrentVelU i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _fUserETT
     (dotimes (i 6)
       (sys::poke (elt _fUserETT i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _fTargetPosU
     (dotimes (i 6)
       (sys::poke (elt _fTargetPosU i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _fTargetVelU
     (dotimes (i 6)
       (sys::poke (elt _fTargetVelU i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; std_msgs/Float64MultiArray[] _fRotationMatrixUser
     (write-long (length _fRotationMatrixUser) s)
     (dolist (elem _fRotationMatrixUser)
       (send elem :serialize s)
       )
     ;; float64[6] _fActualAI
     (dotimes (i 6)
       (sys::poke (elt _fActualAI i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool[3] _bActualSW
     (dotimes (i 3)
       (if (elt _bActualSW i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; bool[2] _bActualSI
     (dotimes (i 2)
       (if (elt _bActualSI i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; int8[2] _iActualAT
     (dotimes (i 2)
       (write-byte (elt _iActualAT i) s)
       )
     ;; float64[2] _fTargetAO
     (dotimes (i 2)
       (sys::poke (elt _fTargetAO i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8[2] _iTargetAT
     (dotimes (i 2)
       (write-byte (elt _iTargetAT i) s)
       )
     ;; bool[2] _bActualES
     (dotimes (i 2)
       (if (elt _bActualES i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; int8[2] _iActualED
     (dotimes (i 2)
       (write-byte (elt _iActualED i) s)
       )
     ;; bool[2] _bActualER
     (dotimes (i 2)
       (if (elt _bActualER i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _robot_state
     (setq _robot_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _robot_state_str
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_state_str (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _actual_mode
     (setq _actual_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _actual_mode 127) (setq _actual_mode (- _actual_mode 256)))
   ;; int8 _actual_space
     (setq _actual_space (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _actual_space 127) (setq _actual_space (- _actual_space 256)))
   ;; float64[6] _current_posj
   (dotimes (i (length _current_posj))
     (setf (elt _current_posj i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _current_velj
   (dotimes (i (length _current_velj))
     (setf (elt _current_velj i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _joint_abs
   (dotimes (i (length _joint_abs))
     (setf (elt _joint_abs i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _joint_err
   (dotimes (i (length _joint_err))
     (setf (elt _joint_err i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _target_posj
   (dotimes (i (length _target_posj))
     (setf (elt _target_posj i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _target_velj
   (dotimes (i (length _target_velj))
     (setf (elt _target_velj i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _current_posx
   (dotimes (i (length _current_posx))
     (setf (elt _current_posx i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _current_tool_posx
   (dotimes (i (length _current_tool_posx))
     (setf (elt _current_tool_posx i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _current_velx
   (dotimes (i (length _current_velx))
     (setf (elt _current_velx i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _task_err
   (dotimes (i (length _task_err))
     (setf (elt _task_err i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _target_velx
   (dotimes (i (length _target_velx))
     (setf (elt _target_velx i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _target_posx
   (dotimes (i (length _target_posx))
     (setf (elt _target_posx i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _dynamic_tor
   (dotimes (i (length _dynamic_tor))
     (setf (elt _dynamic_tor i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _actual_jts
   (dotimes (i (length _actual_jts))
     (setf (elt _actual_jts i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _actual_ejt
   (dotimes (i (length _actual_ejt))
     (setf (elt _actual_ejt i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _actual_ett
   (dotimes (i (length _actual_ett))
     (setf (elt _actual_ett i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int8[6] _actual_bk
   (dotimes (i (length _actual_bk))
     (setf (elt _actual_bk i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _actual_bk i) 127) (setf (elt _actual_bk i) (- (elt _actual_bk i) 256)))
     )
   ;; float64[6] _actual_mc
   (dotimes (i (length _actual_mc))
     (setf (elt _actual_mc i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _actual_mt
   (dotimes (i (length _actual_mt))
     (setf (elt _actual_mt i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int8 _solution_space
     (setq _solution_space (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _solution_space 127) (setq _solution_space (- _solution_space 256)))
   ;; float64 _sync_time
     (setq _sync_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8[5] _actual_bt
   (dotimes (i (length _actual_bt))
     (setf (elt _actual_bt i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _actual_bt i) 127) (setf (elt _actual_bt i) (- (elt _actual_bt i) 256)))
     )
   ;; std_msgs/Float64MultiArray[] _rotation_matrix
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rotation_matrix (let (r) (dotimes (i n) (push (instance std_msgs::Float64MultiArray :init) r)) r))
     (dolist (elem- _rotation_matrix)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int8[16] _ctrlbox_digital_input
   (dotimes (i (length _ctrlbox_digital_input))
     (setf (elt _ctrlbox_digital_input i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _ctrlbox_digital_input i) 127) (setf (elt _ctrlbox_digital_input i) (- (elt _ctrlbox_digital_input i) 256)))
     )
   ;; int8[16] _ctrlbox_digital_output
   (dotimes (i (length _ctrlbox_digital_output))
     (setf (elt _ctrlbox_digital_output i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _ctrlbox_digital_output i) 127) (setf (elt _ctrlbox_digital_output i) (- (elt _ctrlbox_digital_output i) 256)))
     )
   ;; int8[6] _flange_digital_input
   (dotimes (i (length _flange_digital_input))
     (setf (elt _flange_digital_input i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _flange_digital_input i) 127) (setf (elt _flange_digital_input i) (- (elt _flange_digital_input i) 256)))
     )
   ;; int8[6] _flange_digital_output
   (dotimes (i (length _flange_digital_output))
     (setf (elt _flange_digital_output i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _flange_digital_output i) 127) (setf (elt _flange_digital_output i) (- (elt _flange_digital_output i) 256)))
     )
   ;; dsr_msgs/ModbusState[] _modbus_state
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _modbus_state (let (r) (dotimes (i n) (push (instance dsr_msgs::ModbusState :init) r)) r))
     (dolist (elem- _modbus_state)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32 _access_control
     (setq _access_control (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _homming_completed
     (setq _homming_completed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _tp_initialized
     (setq _tp_initialized (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int8 _mastering_need
     (setq _mastering_need (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mastering_need 127) (setq _mastering_need (- _mastering_need 256)))
   ;; bool _drl_stopped
     (setq _drl_stopped (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _disconnected
     (setq _disconnected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64[6] _fActualW2B
   (dotimes (i (length _fActualW2B))
     (setf (elt _fActualW2B i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; std_msgs/Float64MultiArray[] _fCurrentPosW
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _fCurrentPosW (let (r) (dotimes (i n) (push (instance std_msgs::Float64MultiArray :init) r)) r))
     (dolist (elem- _fCurrentPosW)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[6] _fCurrentVelW
   (dotimes (i (length _fCurrentVelW))
     (setf (elt _fCurrentVelW i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _fWorldETT
   (dotimes (i (length _fWorldETT))
     (setf (elt _fWorldETT i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _fTargetPosW
   (dotimes (i (length _fTargetPosW))
     (setf (elt _fTargetPosW i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _fTargetVelW
   (dotimes (i (length _fTargetVelW))
     (setf (elt _fTargetVelW i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; std_msgs/Float64MultiArray[] _fRotationMatrixWorld
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _fRotationMatrixWorld (let (r) (dotimes (i n) (push (instance std_msgs::Float64MultiArray :init) r)) r))
     (dolist (elem- _fRotationMatrixWorld)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int8 _iActualUCN
     (setq _iActualUCN (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _iActualUCN 127) (setq _iActualUCN (- _iActualUCN 256)))
   ;; int8 _iParent
     (setq _iParent (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _iParent 127) (setq _iParent (- _iParent 256)))
   ;; std_msgs/Float64MultiArray[] _fCurrentPosU
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _fCurrentPosU (let (r) (dotimes (i n) (push (instance std_msgs::Float64MultiArray :init) r)) r))
     (dolist (elem- _fCurrentPosU)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[6] _fCurrentVelU
   (dotimes (i (length _fCurrentVelU))
     (setf (elt _fCurrentVelU i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _fUserETT
   (dotimes (i (length _fUserETT))
     (setf (elt _fUserETT i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _fTargetPosU
   (dotimes (i (length _fTargetPosU))
     (setf (elt _fTargetPosU i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _fTargetVelU
   (dotimes (i (length _fTargetVelU))
     (setf (elt _fTargetVelU i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; std_msgs/Float64MultiArray[] _fRotationMatrixUser
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _fRotationMatrixUser (let (r) (dotimes (i n) (push (instance std_msgs::Float64MultiArray :init) r)) r))
     (dolist (elem- _fRotationMatrixUser)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[6] _fActualAI
   (dotimes (i (length _fActualAI))
     (setf (elt _fActualAI i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool[3] _bActualSW
   (dotimes (i (length _bActualSW))
     (setf (elt _bActualSW i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; bool[2] _bActualSI
   (dotimes (i (length _bActualSI))
     (setf (elt _bActualSI i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; int8[2] _iActualAT
   (dotimes (i (length _iActualAT))
     (setf (elt _iActualAT i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _iActualAT i) 127) (setf (elt _iActualAT i) (- (elt _iActualAT i) 256)))
     )
   ;; float64[2] _fTargetAO
   (dotimes (i (length _fTargetAO))
     (setf (elt _fTargetAO i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int8[2] _iTargetAT
   (dotimes (i (length _iTargetAT))
     (setf (elt _iTargetAT i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _iTargetAT i) 127) (setf (elt _iTargetAT i) (- (elt _iTargetAT i) 256)))
     )
   ;; bool[2] _bActualES
   (dotimes (i (length _bActualES))
     (setf (elt _bActualES i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; int8[2] _iActualED
   (dotimes (i (length _iActualED))
     (setf (elt _iActualED i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _iActualED i) 127) (setf (elt _iActualED i) (- (elt _iActualED i) 256)))
     )
   ;; bool[2] _bActualER
   (dotimes (i (length _bActualER))
     (setf (elt _bActualER i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;;
   self)
  )

(setf (get dsr_msgs::RobotState :md5sum-) "0473bf35fc3b2d88cf36052d2ba4677e")
(setf (get dsr_msgs::RobotState :datatype-) "dsr_msgs/RobotState")
(setf (get dsr_msgs::RobotState :definition-)
      "#____________________________________________________________________________________________
# state of robot
#____________________________________________________________________________________________

int32       robot_state             # 0 : STATE_INITIALIZING,   1 : STATE_STANDBY,   2 : STATE_MOVING,    3 : STATE_SAFE_OFF
                                    # 4 : STATE_TEACHING,       5 : STATE_SAFE_STOP, 6 : STATE_EMERGENCY_STOP,
                                    # 7 : STATE_EMERGENCY_STOP, 8 : STATE_HOMMING,   9 : STATE_RECOVERY,  10: STATE_SAFE_STOP2,     
                                    # 11: STATE_SAFE_OFF2,      12: STATE_RESERVED1, 13: STATE_RESERVED2, 14: STATE_RESERVED3,
                                    # 15: STATE_NOT_READY       16: STATE_LAST
string      robot_state_str         # Convert robot_state id to string

int8        actual_mode             # position control: 0, torque control: 1
int8        actual_space            # joint space: 0, task space: 1

float64[6]  current_posj            # current joint angle list [degree] 
float64[6]  current_velj            # current joint velocity list []
float64[6]  joint_abs               # Position Actual Value in ABS
float64[6]  joint_err               # Joint Error
float64[6]  target_posj             # target joint angle list [degree] 
float64[6]  target_velj             # target joint velocity list []

float64[6]  current_posx            # current task angle list []
float64[6]  current_tool_posx       # current task tool angle list []
float64[6]  current_velx            # current task velocity list []
float64[6]  task_err                # Task Error
float64[6]  target_velx             # target task velocity list []       
float64[6]  target_posx             # target task position list []
  
float64[6]  dynamic_tor             # dynamic torque
float64[6]  actual_jts              # joint torque sensor
float64[6]  actual_ejt              # external joint torque
float64[6]  actual_ett              # external tool torque

int8[6]     actual_bk               # brake status 
float64[6]  actual_mc               # motor current 
float64[6]  actual_mt               # motor temperature

int8        solution_space          # Solution Space (0 ~ 7)
float64     sync_time               # internal clock counter   
int8[5]     actual_bt               # cockpit(robot button) info.
std_msgs/Float64MultiArray[] rotation_matrix  # Rotation Matrix [3][3]


int8[16]    ctrlbox_digital_input   # Digital Input in Control Box(0 ~ 15 ; 0 : ON, 1 : OFF)
int8[16]    ctrlbox_digital_output  # Digital Output in Control Box(0 ~ 15 ; 0 : ON, 1 : OFF)

int8[6]     flange_digital_input    # Digital Input in Flange(0 ~ 5 ; 0 : ON, 1 : OFF) x1 port : 0 ~ 2, x2 port : 3 ~ 5
int8[6]     flange_digital_output   # Digital Output in Flange(0 ~ 5 ; 0 : ON, 1 : OFF)

ModbusState[] modbus_state          # Custom msg for modbus state(refer to ModbusState.msg)
int32 	    access_control          # 
bool	    homming_completed       #
bool	    tp_initialized          #
int8	    mastering_need          #
bool	    drl_stopped             #
bool	    disconnected            #

#____________________________________________________________________________________________
# The following messages have been updated since version M2.50 or higher.
#____________________________________________________________________________________________
float64[6]  fActualW2B                               # world to base releation
std_msgs/Float64MultiArray[] fCurrentPosW            # Wolrd position actual value [2][6] : (0: tool, 1: flange) [mm, degree] 
float64[6]  fCurrentVelW                             # World velocity Actual Value [mm/sec, degree/sec]
float64[6]  fWorldETT                                # External Task Force/Torque [N, Nm]
float64[6]  fTargetPosW                              # World target Position [mm, degree]
float64[6]  fTargetVelW                              # World target Velocity [mm/sec, degree/sec]
std_msgs/Float64MultiArray[] fRotationMatrixWorld    # World rotation matrix [3][3]
int8        iActualUCN                               # Actual user coord number ## 101 ~ 120
int8        iParent                                  # Coordinate Reference(base : 0  world : 2)
std_msgs/Float64MultiArray[] fCurrentPosU            # User position Actual Value [2][6] : (0:tool, 1:flange) [mm, degree]
float64[6]  fCurrentVelU                             # User velocity Actual Value [mm/sec, degree/sec]
float64[6]  fUserETT                                 # External Task Force/Torque [N, Nm]
float64[6]  fTargetPosU                              # User target Position [mm, degree]
float64[6]  fTargetVelU                              # User target Velocity [mm/sec, degree/sec]
std_msgs/Float64MultiArray[] fRotationMatrixUser     # User rotation matrix [3][3] 
float64[6]  fActualAI                                # Analog input data ## Current mode : 0~20.0[mA] , Voltage mode : 0~10.0[V]
bool[3]     bActualSW                                # Switch input data[3] 
bool[2]     bActualSI                                # Safety input data[2]
int8[2]     iActualAT                                # Analog input type[2]  index = channel, type: current(0), voltage(1)  
float64[2]    fTargetAO                              # Analog output data ## Current mode : 0~20.0[mA] , Voltage mode : 0~10.0[V]
int8[2]     iTargetAT                                # Analog output type[2] index = channel, type: current(0), voltage(1)
bool[2]     bActualES                                # Encorder strove signal
int8[2]     iActualED                                # Encorder raw data   
bool[2]     bActualER                                # Encorder reset signal

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
================================================================================
MSG: dsr_msgs/ModbusState
#____________________________________________________________________________________________
#Custom msg for RobotState.msg -- MAX_SIZE = 100
#____________________________________________________________________________________________

string  modbus_symbol    # Modbus Signal Name
int32   modbus_value     # Modbus Register Value (Unsigned : 0 ~ 65535)
")



(provide :dsr_msgs/RobotState "0473bf35fc3b2d88cf36052d2ba4677e")


