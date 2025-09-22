;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetRobotMode)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetRobotMode (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETROBOTMODE")
  (make-package "DSR_MSGS::SETROBOTMODE"))
(unless (find-package "DSR_MSGS::SETROBOTMODEREQUEST")
  (make-package "DSR_MSGS::SETROBOTMODEREQUEST"))
(unless (find-package "DSR_MSGS::SETROBOTMODERESPONSE")
  (make-package "DSR_MSGS::SETROBOTMODERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetRobotModeRequest
  :super ros::object
  :slots (_robot_mode ))

(defmethod dsr_msgs::SetRobotModeRequest
  (:init
   (&key
    ((:robot_mode __robot_mode) 0)
    )
   (send-super :init)
   (setq _robot_mode (round __robot_mode))
   self)
  (:robot_mode
   (&optional __robot_mode)
   (if __robot_mode (setq _robot_mode __robot_mode)) _robot_mode)
  (:serialization-length
   ()
   (+
    ;; int8 _robot_mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _robot_mode
       (write-byte _robot_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _robot_mode
     (setq _robot_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _robot_mode 127) (setq _robot_mode (- _robot_mode 256)))
   ;;
   self)
  )

(defclass dsr_msgs::SetRobotModeResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetRobotModeResponse
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

(defclass dsr_msgs::SetRobotMode
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetRobotMode :md5sum-) "1839172555f03e6135521c0cb3b452f9")
(setf (get dsr_msgs::SetRobotMode :datatype-) "dsr_msgs/SetRobotMode")
(setf (get dsr_msgs::SetRobotMode :request) dsr_msgs::SetRobotModeRequest)
(setf (get dsr_msgs::SetRobotMode :response) dsr_msgs::SetRobotModeResponse)

(defmethod dsr_msgs::SetRobotModeRequest
  (:response () (instance dsr_msgs::SetRobotModeResponse :init)))

(setf (get dsr_msgs::SetRobotModeRequest :md5sum-) "1839172555f03e6135521c0cb3b452f9")
(setf (get dsr_msgs::SetRobotModeRequest :datatype-) "dsr_msgs/SetRobotModeRequest")
(setf (get dsr_msgs::SetRobotModeRequest :definition-)
      "#____________________________________________________________________________________________
# set_robot_mode
# Change the robot-mode
# 0 : ROBOT_MODE_MANUAL
# 1 : ROBOT_MODE_AUTONOMOUS
# 2 :ROBOT_MODE_MEASURE
# drfl.SetRobotMode()
#____________________________________________________________________________________________

int8 robot_mode # <Robot_Mode>
---
bool success

")

(setf (get dsr_msgs::SetRobotModeResponse :md5sum-) "1839172555f03e6135521c0cb3b452f9")
(setf (get dsr_msgs::SetRobotModeResponse :datatype-) "dsr_msgs/SetRobotModeResponse")
(setf (get dsr_msgs::SetRobotModeResponse :definition-)
      "#____________________________________________________________________________________________
# set_robot_mode
# Change the robot-mode
# 0 : ROBOT_MODE_MANUAL
# 1 : ROBOT_MODE_AUTONOMOUS
# 2 :ROBOT_MODE_MEASURE
# drfl.SetRobotMode()
#____________________________________________________________________________________________

int8 robot_mode # <Robot_Mode>
---
bool success

")



(provide :dsr_msgs/SetRobotMode "1839172555f03e6135521c0cb3b452f9")


