;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetRobotControl)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetRobotControl (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETROBOTCONTROL")
  (make-package "DSR_MSGS::SETROBOTCONTROL"))
(unless (find-package "DSR_MSGS::SETROBOTCONTROLREQUEST")
  (make-package "DSR_MSGS::SETROBOTCONTROLREQUEST"))
(unless (find-package "DSR_MSGS::SETROBOTCONTROLRESPONSE")
  (make-package "DSR_MSGS::SETROBOTCONTROLRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetRobotControlRequest
  :super ros::object
  :slots (_robot_control ))

(defmethod dsr_msgs::SetRobotControlRequest
  (:init
   (&key
    ((:robot_control __robot_control) 0)
    )
   (send-super :init)
   (setq _robot_control (round __robot_control))
   self)
  (:robot_control
   (&optional __robot_control)
   (if __robot_control (setq _robot_control __robot_control)) _robot_control)
  (:serialization-length
   ()
   (+
    ;; int8 _robot_control
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _robot_control
       (write-byte _robot_control s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _robot_control
     (setq _robot_control (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _robot_control 127) (setq _robot_control (- _robot_control 256)))
   ;;
   self)
  )

(defclass dsr_msgs::SetRobotControlResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetRobotControlResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::SetRobotControl
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetRobotControl :md5sum-) "e3c7c3cfa0120a8cf83abd30ec7bcbc3")
(setf (get dsr_msgs::SetRobotControl :datatype-) "dsr_msgs/SetRobotControl")
(setf (get dsr_msgs::SetRobotControl :request) dsr_msgs::SetRobotControlRequest)
(setf (get dsr_msgs::SetRobotControl :response) dsr_msgs::SetRobotControlResponse)

(defmethod dsr_msgs::SetRobotControlRequest
  (:response () (instance dsr_msgs::SetRobotControlResponse :init)))

(setf (get dsr_msgs::SetRobotControlRequest :md5sum-) "e3c7c3cfa0120a8cf83abd30ec7bcbc3")
(setf (get dsr_msgs::SetRobotControlRequest :datatype-) "dsr_msgs/SetRobotControlRequest")
(setf (get dsr_msgs::SetRobotControlRequest :definition-)
      "#____________________________________________________________________________________________
# set_robot_control
#____________________________________________________________________________________________

int8 robot_control # 0 : CONTROL_INIT_CONFIG
                   # 1 : CONTROL_ENABLE_OPERATION
                   # 2 : CONTROL_RESET_SAFET_STOP
                   # 3 : CONTROL_RESET_SAFE_STOP = CONTROL_RESET_SAFET_STOP,
                   # 4 : CONTROL_RESET_SAFET_OFF,
                   # 5 : CONTROL_RESET_SAFE_OFF = CONTROL_RESET_SAFET_OFF,
                   # 6 : CONTROL_SERVO_ON = CONTROL_RESET_SAFET_OFF,
                   # 7 : CONTROL_RECOVERY_SAFE_STOP,
                   # 8 : CONTROL_RECOVERY_SAFE_OFF,
                   # 9 : CONTROL_RECOVERY_BACKDRIVE,
                   # 10: CONTROL_RESET_RECOVERY,
                   # 11: CONTROL_LAST
---
bool success    
                   

")

(setf (get dsr_msgs::SetRobotControlResponse :md5sum-) "e3c7c3cfa0120a8cf83abd30ec7bcbc3")
(setf (get dsr_msgs::SetRobotControlResponse :datatype-) "dsr_msgs/SetRobotControlResponse")
(setf (get dsr_msgs::SetRobotControlResponse :definition-)
      "#____________________________________________________________________________________________
# set_robot_control
#____________________________________________________________________________________________

int8 robot_control # 0 : CONTROL_INIT_CONFIG
                   # 1 : CONTROL_ENABLE_OPERATION
                   # 2 : CONTROL_RESET_SAFET_STOP
                   # 3 : CONTROL_RESET_SAFE_STOP = CONTROL_RESET_SAFET_STOP,
                   # 4 : CONTROL_RESET_SAFET_OFF,
                   # 5 : CONTROL_RESET_SAFE_OFF = CONTROL_RESET_SAFET_OFF,
                   # 6 : CONTROL_SERVO_ON = CONTROL_RESET_SAFET_OFF,
                   # 7 : CONTROL_RECOVERY_SAFE_STOP,
                   # 8 : CONTROL_RECOVERY_SAFE_OFF,
                   # 9 : CONTROL_RECOVERY_BACKDRIVE,
                   # 10: CONTROL_RESET_RECOVERY,
                   # 11: CONTROL_LAST
---
bool success    
                   

")



(provide :dsr_msgs/SetRobotControl "e3c7c3cfa0120a8cf83abd30ec7bcbc3")


