;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetRobotSystem)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetRobotSystem (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETROBOTSYSTEM")
  (make-package "DSR_MSGS::SETROBOTSYSTEM"))
(unless (find-package "DSR_MSGS::SETROBOTSYSTEMREQUEST")
  (make-package "DSR_MSGS::SETROBOTSYSTEMREQUEST"))
(unless (find-package "DSR_MSGS::SETROBOTSYSTEMRESPONSE")
  (make-package "DSR_MSGS::SETROBOTSYSTEMRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetRobotSystemRequest
  :super ros::object
  :slots (_robot_system ))

(defmethod dsr_msgs::SetRobotSystemRequest
  (:init
   (&key
    ((:robot_system __robot_system) 0)
    )
   (send-super :init)
   (setq _robot_system (round __robot_system))
   self)
  (:robot_system
   (&optional __robot_system)
   (if __robot_system (setq _robot_system __robot_system)) _robot_system)
  (:serialization-length
   ()
   (+
    ;; int8 _robot_system
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _robot_system
       (write-byte _robot_system s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _robot_system
     (setq _robot_system (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _robot_system 127) (setq _robot_system (- _robot_system 256)))
   ;;
   self)
  )

(defclass dsr_msgs::SetRobotSystemResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetRobotSystemResponse
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

(defclass dsr_msgs::SetRobotSystem
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetRobotSystem :md5sum-) "5474fc13fdf01710b0f371e4b3b11c86")
(setf (get dsr_msgs::SetRobotSystem :datatype-) "dsr_msgs/SetRobotSystem")
(setf (get dsr_msgs::SetRobotSystem :request) dsr_msgs::SetRobotSystemRequest)
(setf (get dsr_msgs::SetRobotSystem :response) dsr_msgs::SetRobotSystemResponse)

(defmethod dsr_msgs::SetRobotSystemRequest
  (:response () (instance dsr_msgs::SetRobotSystemResponse :init)))

(setf (get dsr_msgs::SetRobotSystemRequest :md5sum-) "5474fc13fdf01710b0f371e4b3b11c86")
(setf (get dsr_msgs::SetRobotSystemRequest :datatype-) "dsr_msgs/SetRobotSystemRequest")
(setf (get dsr_msgs::SetRobotSystemRequest :definition-)
      "#____________________________________________________________________________________________
# set_robot_system
#____________________________________________________________________________________________

int8 robot_system   # 0 : ROBOT_SYSTEM_REAL, 1 : ROBOT_SYSTEM_VIRTUAL
---
bool success
")

(setf (get dsr_msgs::SetRobotSystemResponse :md5sum-) "5474fc13fdf01710b0f371e4b3b11c86")
(setf (get dsr_msgs::SetRobotSystemResponse :datatype-) "dsr_msgs/SetRobotSystemResponse")
(setf (get dsr_msgs::SetRobotSystemResponse :definition-)
      "#____________________________________________________________________________________________
# set_robot_system
#____________________________________________________________________________________________

int8 robot_system   # 0 : ROBOT_SYSTEM_REAL, 1 : ROBOT_SYSTEM_VIRTUAL
---
bool success
")



(provide :dsr_msgs/SetRobotSystem "5474fc13fdf01710b0f371e4b3b11c86")


