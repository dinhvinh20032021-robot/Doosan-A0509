;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetAccXRT)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetAccXRT (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETACCXRT")
  (make-package "DSR_MSGS::SETACCXRT"))
(unless (find-package "DSR_MSGS::SETACCXRTREQUEST")
  (make-package "DSR_MSGS::SETACCXRTREQUEST"))
(unless (find-package "DSR_MSGS::SETACCXRTRESPONSE")
  (make-package "DSR_MSGS::SETACCXRTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetAccXRTRequest
  :super ros::object
  :slots (_trans _rotation ))

(defmethod dsr_msgs::SetAccXRTRequest
  (:init
   (&key
    ((:trans __trans) 0.0)
    ((:rotation __rotation) 0.0)
    )
   (send-super :init)
   (setq _trans (float __trans))
   (setq _rotation (float __rotation))
   self)
  (:trans
   (&optional __trans)
   (if __trans (setq _trans __trans)) _trans)
  (:rotation
   (&optional __rotation)
   (if __rotation (setq _rotation __rotation)) _rotation)
  (:serialization-length
   ()
   (+
    ;; float64 _trans
    8
    ;; float64 _rotation
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _trans
       (sys::poke _trans (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rotation
       (sys::poke _rotation (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _trans
     (setq _trans (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rotation
     (setq _rotation (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass dsr_msgs::SetAccXRTResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetAccXRTResponse
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

(defclass dsr_msgs::SetAccXRT
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetAccXRT :md5sum-) "761d85a527beb96265a49a9c48c83232")
(setf (get dsr_msgs::SetAccXRT :datatype-) "dsr_msgs/SetAccXRT")
(setf (get dsr_msgs::SetAccXRT :request) dsr_msgs::SetAccXRTRequest)
(setf (get dsr_msgs::SetAccXRT :response) dsr_msgs::SetAccXRTResponse)

(defmethod dsr_msgs::SetAccXRTRequest
  (:response () (instance dsr_msgs::SetAccXRTResponse :init)))

(setf (get dsr_msgs::SetAccXRTRequest :md5sum-) "761d85a527beb96265a49a9c48c83232")
(setf (get dsr_msgs::SetAccXRTRequest :datatype-) "dsr_msgs/SetAccXRTRequest")
(setf (get dsr_msgs::SetAccXRTRequest :definition-)
      "#____________________________________________________________________________________________
# set_accx_rt
#____________________________________________________________________________________________
float64    trans
float64    rotation
---
bool       success
")

(setf (get dsr_msgs::SetAccXRTResponse :md5sum-) "761d85a527beb96265a49a9c48c83232")
(setf (get dsr_msgs::SetAccXRTResponse :datatype-) "dsr_msgs/SetAccXRTResponse")
(setf (get dsr_msgs::SetAccXRTResponse :definition-)
      "#____________________________________________________________________________________________
# set_accx_rt
#____________________________________________________________________________________________
float64    trans
float64    rotation
---
bool       success
")



(provide :dsr_msgs/SetAccXRT "761d85a527beb96265a49a9c48c83232")


