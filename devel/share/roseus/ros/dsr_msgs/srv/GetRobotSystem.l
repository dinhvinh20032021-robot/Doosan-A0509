;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetRobotSystem)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetRobotSystem (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETROBOTSYSTEM")
  (make-package "DSR_MSGS::GETROBOTSYSTEM"))
(unless (find-package "DSR_MSGS::GETROBOTSYSTEMREQUEST")
  (make-package "DSR_MSGS::GETROBOTSYSTEMREQUEST"))
(unless (find-package "DSR_MSGS::GETROBOTSYSTEMRESPONSE")
  (make-package "DSR_MSGS::GETROBOTSYSTEMRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetRobotSystemRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetRobotSystemRequest
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

(defclass dsr_msgs::GetRobotSystemResponse
  :super ros::object
  :slots (_robot_system _success ))

(defmethod dsr_msgs::GetRobotSystemResponse
  (:init
   (&key
    ((:robot_system __robot_system) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _robot_system (round __robot_system))
   (setq _success __success)
   self)
  (:robot_system
   (&optional __robot_system)
   (if __robot_system (setq _robot_system __robot_system)) _robot_system)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int8 _robot_system
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _robot_system
       (write-byte _robot_system s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _robot_system
     (setq _robot_system (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _robot_system 127) (setq _robot_system (- _robot_system 256)))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetRobotSystem
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetRobotSystem :md5sum-) "20308dcb70c4e3f1df099ac95a676cf2")
(setf (get dsr_msgs::GetRobotSystem :datatype-) "dsr_msgs/GetRobotSystem")
(setf (get dsr_msgs::GetRobotSystem :request) dsr_msgs::GetRobotSystemRequest)
(setf (get dsr_msgs::GetRobotSystem :response) dsr_msgs::GetRobotSystemResponse)

(defmethod dsr_msgs::GetRobotSystemRequest
  (:response () (instance dsr_msgs::GetRobotSystemResponse :init)))

(setf (get dsr_msgs::GetRobotSystemRequest :md5sum-) "20308dcb70c4e3f1df099ac95a676cf2")
(setf (get dsr_msgs::GetRobotSystemRequest :datatype-) "dsr_msgs/GetRobotSystemRequest")
(setf (get dsr_msgs::GetRobotSystemRequest :definition-)
      "#____________________________________________________________________________________________
# get_robot_system
#____________________________________________________________________________________________

---
int8 robot_system   # 0 : ROBOT_SYSTEM_REAL
                    # 1 : ROBOT_SYSTEM_VIRTUAL
bool        success
")

(setf (get dsr_msgs::GetRobotSystemResponse :md5sum-) "20308dcb70c4e3f1df099ac95a676cf2")
(setf (get dsr_msgs::GetRobotSystemResponse :datatype-) "dsr_msgs/GetRobotSystemResponse")
(setf (get dsr_msgs::GetRobotSystemResponse :definition-)
      "#____________________________________________________________________________________________
# get_robot_system
#____________________________________________________________________________________________

---
int8 robot_system   # 0 : ROBOT_SYSTEM_REAL
                    # 1 : ROBOT_SYSTEM_VIRTUAL
bool        success
")



(provide :dsr_msgs/GetRobotSystem "20308dcb70c4e3f1df099ac95a676cf2")


