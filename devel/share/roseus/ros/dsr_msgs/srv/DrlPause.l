;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::DrlPause)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'DrlPause (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::DRLPAUSE")
  (make-package "DSR_MSGS::DRLPAUSE"))
(unless (find-package "DSR_MSGS::DRLPAUSEREQUEST")
  (make-package "DSR_MSGS::DRLPAUSEREQUEST"))
(unless (find-package "DSR_MSGS::DRLPAUSERESPONSE")
  (make-package "DSR_MSGS::DRLPAUSERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::DrlPauseRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::DrlPauseRequest
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

(defclass dsr_msgs::DrlPauseResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::DrlPauseResponse
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

(defclass dsr_msgs::DrlPause
  :super ros::object
  :slots ())

(setf (get dsr_msgs::DrlPause :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::DrlPause :datatype-) "dsr_msgs/DrlPause")
(setf (get dsr_msgs::DrlPause :request) dsr_msgs::DrlPauseRequest)
(setf (get dsr_msgs::DrlPause :response) dsr_msgs::DrlPauseResponse)

(defmethod dsr_msgs::DrlPauseRequest
  (:response () (instance dsr_msgs::DrlPauseResponse :init)))

(setf (get dsr_msgs::DrlPauseRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::DrlPauseRequest :datatype-) "dsr_msgs/DrlPauseRequest")
(setf (get dsr_msgs::DrlPauseRequest :definition-)
      "#____________________________________________________________________________________________
# drl_script_pause  
# This service is used to stop the currently executing DRL program from the robot controller.
#____________________________________________________________________________________________

---
bool success
")

(setf (get dsr_msgs::DrlPauseResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::DrlPauseResponse :datatype-) "dsr_msgs/DrlPauseResponse")
(setf (get dsr_msgs::DrlPauseResponse :definition-)
      "#____________________________________________________________________________________________
# drl_script_pause  
# This service is used to stop the currently executing DRL program from the robot controller.
#____________________________________________________________________________________________

---
bool success
")



(provide :dsr_msgs/DrlPause "358e233cde0c8a8bcfea4ce193f8fc15")


