;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetRobotMode)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetRobotMode (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETROBOTMODE")
  (make-package "DSR_MSGS::GETROBOTMODE"))
(unless (find-package "DSR_MSGS::GETROBOTMODEREQUEST")
  (make-package "DSR_MSGS::GETROBOTMODEREQUEST"))
(unless (find-package "DSR_MSGS::GETROBOTMODERESPONSE")
  (make-package "DSR_MSGS::GETROBOTMODERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetRobotModeRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetRobotModeRequest
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

(defclass dsr_msgs::GetRobotModeResponse
  :super ros::object
  :slots (_robot_mode _success ))

(defmethod dsr_msgs::GetRobotModeResponse
  (:init
   (&key
    ((:robot_mode __robot_mode) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _robot_mode (round __robot_mode))
   (setq _success __success)
   self)
  (:robot_mode
   (&optional __robot_mode)
   (if __robot_mode (setq _robot_mode __robot_mode)) _robot_mode)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int8 _robot_mode
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _robot_mode
       (write-byte _robot_mode s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _robot_mode
     (setq _robot_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _robot_mode 127) (setq _robot_mode (- _robot_mode 256)))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetRobotMode
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetRobotMode :md5sum-) "020eb3bd52e8053b30ef11386e0bba83")
(setf (get dsr_msgs::GetRobotMode :datatype-) "dsr_msgs/GetRobotMode")
(setf (get dsr_msgs::GetRobotMode :request) dsr_msgs::GetRobotModeRequest)
(setf (get dsr_msgs::GetRobotMode :response) dsr_msgs::GetRobotModeResponse)

(defmethod dsr_msgs::GetRobotModeRequest
  (:response () (instance dsr_msgs::GetRobotModeResponse :init)))

(setf (get dsr_msgs::GetRobotModeRequest :md5sum-) "020eb3bd52e8053b30ef11386e0bba83")
(setf (get dsr_msgs::GetRobotModeRequest :datatype-) "dsr_msgs/GetRobotModeRequest")
(setf (get dsr_msgs::GetRobotModeRequest :definition-)
      "#____________________________________________________________________________________________
# get_robot_mode
# Return to current robot-mode
# 0 : ROBOT_MODE_MANUAL
# 1 : ROBOT_MODE_AUTONOMOUS
# 2 : ROBOT_MODE_MEASURE
# drfl.GetRobotMode()
#____________________________________________________________________________________________

---
int8 robot_mode
bool        success
")

(setf (get dsr_msgs::GetRobotModeResponse :md5sum-) "020eb3bd52e8053b30ef11386e0bba83")
(setf (get dsr_msgs::GetRobotModeResponse :datatype-) "dsr_msgs/GetRobotModeResponse")
(setf (get dsr_msgs::GetRobotModeResponse :definition-)
      "#____________________________________________________________________________________________
# get_robot_mode
# Return to current robot-mode
# 0 : ROBOT_MODE_MANUAL
# 1 : ROBOT_MODE_AUTONOMOUS
# 2 : ROBOT_MODE_MEASURE
# drfl.GetRobotMode()
#____________________________________________________________________________________________

---
int8 robot_mode
bool        success
")



(provide :dsr_msgs/GetRobotMode "020eb3bd52e8053b30ef11386e0bba83")


