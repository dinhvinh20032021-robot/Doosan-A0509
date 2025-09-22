;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::StartRTControl)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'StartRTControl (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::STARTRTCONTROL")
  (make-package "DSR_MSGS::STARTRTCONTROL"))
(unless (find-package "DSR_MSGS::STARTRTCONTROLREQUEST")
  (make-package "DSR_MSGS::STARTRTCONTROLREQUEST"))
(unless (find-package "DSR_MSGS::STARTRTCONTROLRESPONSE")
  (make-package "DSR_MSGS::STARTRTCONTROLRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::StartRTControlRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::StartRTControlRequest
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

(defclass dsr_msgs::StartRTControlResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::StartRTControlResponse
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

(defclass dsr_msgs::StartRTControl
  :super ros::object
  :slots ())

(setf (get dsr_msgs::StartRTControl :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::StartRTControl :datatype-) "dsr_msgs/StartRTControl")
(setf (get dsr_msgs::StartRTControl :request) dsr_msgs::StartRTControlRequest)
(setf (get dsr_msgs::StartRTControl :response) dsr_msgs::StartRTControlResponse)

(defmethod dsr_msgs::StartRTControlRequest
  (:response () (instance dsr_msgs::StartRTControlResponse :init)))

(setf (get dsr_msgs::StartRTControlRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::StartRTControlRequest :datatype-) "dsr_msgs/StartRTControlRequest")
(setf (get dsr_msgs::StartRTControlRequest :definition-)
      "#____________________________________________________________________________________________
# start_rt_control
#____________________________________________________________________________________________

---
bool       success
")

(setf (get dsr_msgs::StartRTControlResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::StartRTControlResponse :datatype-) "dsr_msgs/StartRTControlResponse")
(setf (get dsr_msgs::StartRTControlResponse :definition-)
      "#____________________________________________________________________________________________
# start_rt_control
#____________________________________________________________________________________________

---
bool       success
")



(provide :dsr_msgs/StartRTControl "358e233cde0c8a8bcfea4ce193f8fc15")


