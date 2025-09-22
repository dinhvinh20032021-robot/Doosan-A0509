;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetRobotSpeedMode)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetRobotSpeedMode (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETROBOTSPEEDMODE")
  (make-package "DSR_MSGS::GETROBOTSPEEDMODE"))
(unless (find-package "DSR_MSGS::GETROBOTSPEEDMODEREQUEST")
  (make-package "DSR_MSGS::GETROBOTSPEEDMODEREQUEST"))
(unless (find-package "DSR_MSGS::GETROBOTSPEEDMODERESPONSE")
  (make-package "DSR_MSGS::GETROBOTSPEEDMODERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetRobotSpeedModeRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetRobotSpeedModeRequest
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

(defclass dsr_msgs::GetRobotSpeedModeResponse
  :super ros::object
  :slots (_speed_mode _success ))

(defmethod dsr_msgs::GetRobotSpeedModeResponse
  (:init
   (&key
    ((:speed_mode __speed_mode) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _speed_mode (round __speed_mode))
   (setq _success __success)
   self)
  (:speed_mode
   (&optional __speed_mode)
   (if __speed_mode (setq _speed_mode __speed_mode)) _speed_mode)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int8 _speed_mode
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _speed_mode
       (write-byte _speed_mode s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _speed_mode
     (setq _speed_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _speed_mode 127) (setq _speed_mode (- _speed_mode 256)))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetRobotSpeedMode
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetRobotSpeedMode :md5sum-) "b5d54772deeae3778247a5930ed4c1f0")
(setf (get dsr_msgs::GetRobotSpeedMode :datatype-) "dsr_msgs/GetRobotSpeedMode")
(setf (get dsr_msgs::GetRobotSpeedMode :request) dsr_msgs::GetRobotSpeedModeRequest)
(setf (get dsr_msgs::GetRobotSpeedMode :response) dsr_msgs::GetRobotSpeedModeResponse)

(defmethod dsr_msgs::GetRobotSpeedModeRequest
  (:response () (instance dsr_msgs::GetRobotSpeedModeResponse :init)))

(setf (get dsr_msgs::GetRobotSpeedModeRequest :md5sum-) "b5d54772deeae3778247a5930ed4c1f0")
(setf (get dsr_msgs::GetRobotSpeedModeRequest :datatype-) "dsr_msgs/GetRobotSpeedModeRequest")
(setf (get dsr_msgs::GetRobotSpeedModeRequest :definition-)
      "#____________________________________________________________________________________________
# get_robot_speed_mode
#____________________________________________________________________________________________

---
int8 speed_mode # 0 : SPEED_NORMAL_MODE
                # 1 : SPEED_REDUCED_MODE
bool        success   
")

(setf (get dsr_msgs::GetRobotSpeedModeResponse :md5sum-) "b5d54772deeae3778247a5930ed4c1f0")
(setf (get dsr_msgs::GetRobotSpeedModeResponse :datatype-) "dsr_msgs/GetRobotSpeedModeResponse")
(setf (get dsr_msgs::GetRobotSpeedModeResponse :definition-)
      "#____________________________________________________________________________________________
# get_robot_speed_mode
#____________________________________________________________________________________________

---
int8 speed_mode # 0 : SPEED_NORMAL_MODE
                # 1 : SPEED_REDUCED_MODE
bool        success   
")



(provide :dsr_msgs/GetRobotSpeedMode "b5d54772deeae3778247a5930ed4c1f0")


