;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetRobotSpeedMode)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetRobotSpeedMode (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETROBOTSPEEDMODE")
  (make-package "DSR_MSGS::SETROBOTSPEEDMODE"))
(unless (find-package "DSR_MSGS::SETROBOTSPEEDMODEREQUEST")
  (make-package "DSR_MSGS::SETROBOTSPEEDMODEREQUEST"))
(unless (find-package "DSR_MSGS::SETROBOTSPEEDMODERESPONSE")
  (make-package "DSR_MSGS::SETROBOTSPEEDMODERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetRobotSpeedModeRequest
  :super ros::object
  :slots (_speed_mode ))

(defmethod dsr_msgs::SetRobotSpeedModeRequest
  (:init
   (&key
    ((:speed_mode __speed_mode) 0)
    )
   (send-super :init)
   (setq _speed_mode (round __speed_mode))
   self)
  (:speed_mode
   (&optional __speed_mode)
   (if __speed_mode (setq _speed_mode __speed_mode)) _speed_mode)
  (:serialization-length
   ()
   (+
    ;; int8 _speed_mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _speed_mode
       (write-byte _speed_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _speed_mode
     (setq _speed_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _speed_mode 127) (setq _speed_mode (- _speed_mode 256)))
   ;;
   self)
  )

(defclass dsr_msgs::SetRobotSpeedModeResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetRobotSpeedModeResponse
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

(defclass dsr_msgs::SetRobotSpeedMode
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetRobotSpeedMode :md5sum-) "724227601ad811bee3732775fc3c79d1")
(setf (get dsr_msgs::SetRobotSpeedMode :datatype-) "dsr_msgs/SetRobotSpeedMode")
(setf (get dsr_msgs::SetRobotSpeedMode :request) dsr_msgs::SetRobotSpeedModeRequest)
(setf (get dsr_msgs::SetRobotSpeedMode :response) dsr_msgs::SetRobotSpeedModeResponse)

(defmethod dsr_msgs::SetRobotSpeedModeRequest
  (:response () (instance dsr_msgs::SetRobotSpeedModeResponse :init)))

(setf (get dsr_msgs::SetRobotSpeedModeRequest :md5sum-) "724227601ad811bee3732775fc3c79d1")
(setf (get dsr_msgs::SetRobotSpeedModeRequest :datatype-) "dsr_msgs/SetRobotSpeedModeRequest")
(setf (get dsr_msgs::SetRobotSpeedModeRequest :definition-)
      "#____________________________________________________________________________________________
# set_robot_speed_mode
#____________________________________________________________________________________________

int8 speed_mode # 0 : SPEED_NORMAL_MODE, 1 : SPEED_REDUCED_MODE
---
bool success    
                   
")

(setf (get dsr_msgs::SetRobotSpeedModeResponse :md5sum-) "724227601ad811bee3732775fc3c79d1")
(setf (get dsr_msgs::SetRobotSpeedModeResponse :datatype-) "dsr_msgs/SetRobotSpeedModeResponse")
(setf (get dsr_msgs::SetRobotSpeedModeResponse :definition-)
      "#____________________________________________________________________________________________
# set_robot_speed_mode
#____________________________________________________________________________________________

int8 speed_mode # 0 : SPEED_NORMAL_MODE, 1 : SPEED_REDUCED_MODE
---
bool success    
                   
")



(provide :dsr_msgs/SetRobotSpeedMode "724227601ad811bee3732775fc3c79d1")


