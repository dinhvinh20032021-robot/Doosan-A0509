;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetRobotState)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetRobotState (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETROBOTSTATE")
  (make-package "DSR_MSGS::GETROBOTSTATE"))
(unless (find-package "DSR_MSGS::GETROBOTSTATEREQUEST")
  (make-package "DSR_MSGS::GETROBOTSTATEREQUEST"))
(unless (find-package "DSR_MSGS::GETROBOTSTATERESPONSE")
  (make-package "DSR_MSGS::GETROBOTSTATERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetRobotStateRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetRobotStateRequest
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

(defclass dsr_msgs::GetRobotStateResponse
  :super ros::object
  :slots (_robot_state _success ))

(defmethod dsr_msgs::GetRobotStateResponse
  (:init
   (&key
    ((:robot_state __robot_state) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _robot_state (round __robot_state))
   (setq _success __success)
   self)
  (:robot_state
   (&optional __robot_state)
   (if __robot_state (setq _robot_state __robot_state)) _robot_state)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int8 _robot_state
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _robot_state
       (write-byte _robot_state s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _robot_state
     (setq _robot_state (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _robot_state 127) (setq _robot_state (- _robot_state 256)))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetRobotState
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetRobotState :md5sum-) "55f09762597f0e81f6f46902de1fad06")
(setf (get dsr_msgs::GetRobotState :datatype-) "dsr_msgs/GetRobotState")
(setf (get dsr_msgs::GetRobotState :request) dsr_msgs::GetRobotStateRequest)
(setf (get dsr_msgs::GetRobotState :response) dsr_msgs::GetRobotStateResponse)

(defmethod dsr_msgs::GetRobotStateRequest
  (:response () (instance dsr_msgs::GetRobotStateResponse :init)))

(setf (get dsr_msgs::GetRobotStateRequest :md5sum-) "55f09762597f0e81f6f46902de1fad06")
(setf (get dsr_msgs::GetRobotStateRequest :datatype-) "dsr_msgs/GetRobotStateRequest")
(setf (get dsr_msgs::GetRobotStateRequest :definition-)
      "#____________________________________________________________________________________________
# get_robot_state
#____________________________________________________________________________________________

---
int8 robot_state    # 0 : STATE_INITIALIZING
                    # 1 : STATE_STANDBY
                    # 2 : STATE_MOVING
                    # 3 : STATE_SAFE_OFF
                    # 4 : STATE_TEACHING
                    # 5 : STATE_SAFE_STOP
                    # 6 : STATE_EMERGENCY_STOP:
                    # 7 : STATE_HOMMING
                    # 8 : STATE_RECOVERY
                    # 9 : eSTATE_SAFE_STOP2
                    # 10: STATE_SAFE_OFF2
                    # 11: STATE_RESERVED1
                    # 12: STATE_RESERVED2
                    # 13: STATE_RESERVED3
                    # 14: STATE_RESERVED4
                    # 15: STATE_NOT_READY
bool        success                    
")

(setf (get dsr_msgs::GetRobotStateResponse :md5sum-) "55f09762597f0e81f6f46902de1fad06")
(setf (get dsr_msgs::GetRobotStateResponse :datatype-) "dsr_msgs/GetRobotStateResponse")
(setf (get dsr_msgs::GetRobotStateResponse :definition-)
      "#____________________________________________________________________________________________
# get_robot_state
#____________________________________________________________________________________________

---
int8 robot_state    # 0 : STATE_INITIALIZING
                    # 1 : STATE_STANDBY
                    # 2 : STATE_MOVING
                    # 3 : STATE_SAFE_OFF
                    # 4 : STATE_TEACHING
                    # 5 : STATE_SAFE_STOP
                    # 6 : STATE_EMERGENCY_STOP:
                    # 7 : STATE_HOMMING
                    # 8 : STATE_RECOVERY
                    # 9 : eSTATE_SAFE_STOP2
                    # 10: STATE_SAFE_OFF2
                    # 11: STATE_RESERVED1
                    # 12: STATE_RESERVED2
                    # 13: STATE_RESERVED3
                    # 14: STATE_RESERVED4
                    # 15: STATE_NOT_READY
bool        success                    
")



(provide :dsr_msgs/GetRobotState "55f09762597f0e81f6f46902de1fad06")


