;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::Robotiq2FMove)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'Robotiq2FMove (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::ROBOTIQ2FMOVE")
  (make-package "DSR_MSGS::ROBOTIQ2FMOVE"))
(unless (find-package "DSR_MSGS::ROBOTIQ2FMOVEREQUEST")
  (make-package "DSR_MSGS::ROBOTIQ2FMOVEREQUEST"))
(unless (find-package "DSR_MSGS::ROBOTIQ2FMOVERESPONSE")
  (make-package "DSR_MSGS::ROBOTIQ2FMOVERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::Robotiq2FMoveRequest
  :super ros::object
  :slots (_width ))

(defmethod dsr_msgs::Robotiq2FMoveRequest
  (:init
   (&key
    ((:width __width) 0.0)
    )
   (send-super :init)
   (setq _width (float __width))
   self)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:serialization-length
   ()
   (+
    ;; float64 _width
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _width
       (sys::poke _width (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _width
     (setq _width (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass dsr_msgs::Robotiq2FMoveResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::Robotiq2FMoveResponse
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

(defclass dsr_msgs::Robotiq2FMove
  :super ros::object
  :slots ())

(setf (get dsr_msgs::Robotiq2FMove :md5sum-) "274da388c4c2130f9c892586a4676f9b")
(setf (get dsr_msgs::Robotiq2FMove :datatype-) "dsr_msgs/Robotiq2FMove")
(setf (get dsr_msgs::Robotiq2FMove :request) dsr_msgs::Robotiq2FMoveRequest)
(setf (get dsr_msgs::Robotiq2FMove :response) dsr_msgs::Robotiq2FMoveResponse)

(defmethod dsr_msgs::Robotiq2FMoveRequest
  (:response () (instance dsr_msgs::Robotiq2FMoveResponse :init)))

(setf (get dsr_msgs::Robotiq2FMoveRequest :md5sum-) "274da388c4c2130f9c892586a4676f9b")
(setf (get dsr_msgs::Robotiq2FMoveRequest :datatype-) "dsr_msgs/Robotiq2FMoveRequest")
(setf (get dsr_msgs::Robotiq2FMoveRequest :definition-)
      "#____________________________________________________________________________________________
# robotiq_2f_move
# open or close the virtual robotiq gripper by input value(width)   
#____________________________________________________________________________________________

float64 width # 0.0(open) ~ 0.8(close)
---
bool success

")

(setf (get dsr_msgs::Robotiq2FMoveResponse :md5sum-) "274da388c4c2130f9c892586a4676f9b")
(setf (get dsr_msgs::Robotiq2FMoveResponse :datatype-) "dsr_msgs/Robotiq2FMoveResponse")
(setf (get dsr_msgs::Robotiq2FMoveResponse :definition-)
      "#____________________________________________________________________________________________
# robotiq_2f_move
# open or close the virtual robotiq gripper by input value(width)   
#____________________________________________________________________________________________

float64 width # 0.0(open) ~ 0.8(close)
---
bool success

")



(provide :dsr_msgs/Robotiq2FMove "274da388c4c2130f9c892586a4676f9b")


