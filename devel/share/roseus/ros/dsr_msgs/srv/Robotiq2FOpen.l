;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::Robotiq2FOpen)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'Robotiq2FOpen (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::ROBOTIQ2FOPEN")
  (make-package "DSR_MSGS::ROBOTIQ2FOPEN"))
(unless (find-package "DSR_MSGS::ROBOTIQ2FOPENREQUEST")
  (make-package "DSR_MSGS::ROBOTIQ2FOPENREQUEST"))
(unless (find-package "DSR_MSGS::ROBOTIQ2FOPENRESPONSE")
  (make-package "DSR_MSGS::ROBOTIQ2FOPENRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::Robotiq2FOpenRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::Robotiq2FOpenRequest
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

(defclass dsr_msgs::Robotiq2FOpenResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::Robotiq2FOpenResponse
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

(defclass dsr_msgs::Robotiq2FOpen
  :super ros::object
  :slots ())

(setf (get dsr_msgs::Robotiq2FOpen :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::Robotiq2FOpen :datatype-) "dsr_msgs/Robotiq2FOpen")
(setf (get dsr_msgs::Robotiq2FOpen :request) dsr_msgs::Robotiq2FOpenRequest)
(setf (get dsr_msgs::Robotiq2FOpen :response) dsr_msgs::Robotiq2FOpenResponse)

(defmethod dsr_msgs::Robotiq2FOpenRequest
  (:response () (instance dsr_msgs::Robotiq2FOpenResponse :init)))

(setf (get dsr_msgs::Robotiq2FOpenRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::Robotiq2FOpenRequest :datatype-) "dsr_msgs/Robotiq2FOpenRequest")
(setf (get dsr_msgs::Robotiq2FOpenRequest :definition-)
      "#____________________________________________________________________________________________
# robotiq_2f_open
# open the virtual robotiq gripper  
#____________________________________________________________________________________________

# None  
---
bool success
")

(setf (get dsr_msgs::Robotiq2FOpenResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::Robotiq2FOpenResponse :datatype-) "dsr_msgs/Robotiq2FOpenResponse")
(setf (get dsr_msgs::Robotiq2FOpenResponse :definition-)
      "#____________________________________________________________________________________________
# robotiq_2f_open
# open the virtual robotiq gripper  
#____________________________________________________________________________________________

# None  
---
bool success
")



(provide :dsr_msgs/Robotiq2FOpen "358e233cde0c8a8bcfea4ce193f8fc15")


