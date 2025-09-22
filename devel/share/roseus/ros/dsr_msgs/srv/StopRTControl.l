;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::StopRTControl)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'StopRTControl (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::STOPRTCONTROL")
  (make-package "DSR_MSGS::STOPRTCONTROL"))
(unless (find-package "DSR_MSGS::STOPRTCONTROLREQUEST")
  (make-package "DSR_MSGS::STOPRTCONTROLREQUEST"))
(unless (find-package "DSR_MSGS::STOPRTCONTROLRESPONSE")
  (make-package "DSR_MSGS::STOPRTCONTROLRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::StopRTControlRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::StopRTControlRequest
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

(defclass dsr_msgs::StopRTControlResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::StopRTControlResponse
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

(defclass dsr_msgs::StopRTControl
  :super ros::object
  :slots ())

(setf (get dsr_msgs::StopRTControl :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::StopRTControl :datatype-) "dsr_msgs/StopRTControl")
(setf (get dsr_msgs::StopRTControl :request) dsr_msgs::StopRTControlRequest)
(setf (get dsr_msgs::StopRTControl :response) dsr_msgs::StopRTControlResponse)

(defmethod dsr_msgs::StopRTControlRequest
  (:response () (instance dsr_msgs::StopRTControlResponse :init)))

(setf (get dsr_msgs::StopRTControlRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::StopRTControlRequest :datatype-) "dsr_msgs/StopRTControlRequest")
(setf (get dsr_msgs::StopRTControlRequest :definition-)
      "#____________________________________________________________________________________________
# stop_rt_control
#____________________________________________________________________________________________
---
bool       success
")

(setf (get dsr_msgs::StopRTControlResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::StopRTControlResponse :datatype-) "dsr_msgs/StopRTControlResponse")
(setf (get dsr_msgs::StopRTControlResponse :definition-)
      "#____________________________________________________________________________________________
# stop_rt_control
#____________________________________________________________________________________________
---
bool       success
")



(provide :dsr_msgs/StopRTControl "358e233cde0c8a8bcfea4ce193f8fc15")


