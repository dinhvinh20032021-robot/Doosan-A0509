;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::DisconnectRTControl)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'DisconnectRTControl (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::DISCONNECTRTCONTROL")
  (make-package "DSR_MSGS::DISCONNECTRTCONTROL"))
(unless (find-package "DSR_MSGS::DISCONNECTRTCONTROLREQUEST")
  (make-package "DSR_MSGS::DISCONNECTRTCONTROLREQUEST"))
(unless (find-package "DSR_MSGS::DISCONNECTRTCONTROLRESPONSE")
  (make-package "DSR_MSGS::DISCONNECTRTCONTROLRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::DisconnectRTControlRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::DisconnectRTControlRequest
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

(defclass dsr_msgs::DisconnectRTControlResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::DisconnectRTControlResponse
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

(defclass dsr_msgs::DisconnectRTControl
  :super ros::object
  :slots ())

(setf (get dsr_msgs::DisconnectRTControl :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::DisconnectRTControl :datatype-) "dsr_msgs/DisconnectRTControl")
(setf (get dsr_msgs::DisconnectRTControl :request) dsr_msgs::DisconnectRTControlRequest)
(setf (get dsr_msgs::DisconnectRTControl :response) dsr_msgs::DisconnectRTControlResponse)

(defmethod dsr_msgs::DisconnectRTControlRequest
  (:response () (instance dsr_msgs::DisconnectRTControlResponse :init)))

(setf (get dsr_msgs::DisconnectRTControlRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::DisconnectRTControlRequest :datatype-) "dsr_msgs/DisconnectRTControlRequest")
(setf (get dsr_msgs::DisconnectRTControlRequest :definition-)
      "#____________________________________________________________________________________________
# disconnect_rt_control
#____________________________________________________________________________________________
---
bool       success
")

(setf (get dsr_msgs::DisconnectRTControlResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::DisconnectRTControlResponse :datatype-) "dsr_msgs/DisconnectRTControlResponse")
(setf (get dsr_msgs::DisconnectRTControlResponse :definition-)
      "#____________________________________________________________________________________________
# disconnect_rt_control
#____________________________________________________________________________________________
---
bool       success
")



(provide :dsr_msgs/DisconnectRTControl "358e233cde0c8a8bcfea4ce193f8fc15")


