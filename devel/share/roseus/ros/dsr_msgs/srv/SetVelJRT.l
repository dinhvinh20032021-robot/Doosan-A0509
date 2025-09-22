;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetVelJRT)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetVelJRT (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETVELJRT")
  (make-package "DSR_MSGS::SETVELJRT"))
(unless (find-package "DSR_MSGS::SETVELJRTREQUEST")
  (make-package "DSR_MSGS::SETVELJRTREQUEST"))
(unless (find-package "DSR_MSGS::SETVELJRTRESPONSE")
  (make-package "DSR_MSGS::SETVELJRTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetVelJRTRequest
  :super ros::object
  :slots (_vel ))

(defmethod dsr_msgs::SetVelJRTRequest
  (:init
   (&key
    ((:vel __vel) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _vel __vel)
   self)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:serialization-length
   ()
   (+
    ;; float64[6] _vel
    (* 8    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _vel
     (dotimes (i 6)
       (sys::poke (elt _vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _vel
   (dotimes (i (length _vel))
     (setf (elt _vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass dsr_msgs::SetVelJRTResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetVelJRTResponse
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

(defclass dsr_msgs::SetVelJRT
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetVelJRT :md5sum-) "54c890dbfd325b76bab7dbd45237c7df")
(setf (get dsr_msgs::SetVelJRT :datatype-) "dsr_msgs/SetVelJRT")
(setf (get dsr_msgs::SetVelJRT :request) dsr_msgs::SetVelJRTRequest)
(setf (get dsr_msgs::SetVelJRT :response) dsr_msgs::SetVelJRTResponse)

(defmethod dsr_msgs::SetVelJRTRequest
  (:response () (instance dsr_msgs::SetVelJRTResponse :init)))

(setf (get dsr_msgs::SetVelJRTRequest :md5sum-) "54c890dbfd325b76bab7dbd45237c7df")
(setf (get dsr_msgs::SetVelJRTRequest :datatype-) "dsr_msgs/SetVelJRTRequest")
(setf (get dsr_msgs::SetVelJRTRequest :definition-)
      "#____________________________________________________________________________________________
# set_velj_rt
#____________________________________________________________________________________________
float64[6] vel
---
bool       success
")

(setf (get dsr_msgs::SetVelJRTResponse :md5sum-) "54c890dbfd325b76bab7dbd45237c7df")
(setf (get dsr_msgs::SetVelJRTResponse :datatype-) "dsr_msgs/SetVelJRTResponse")
(setf (get dsr_msgs::SetVelJRTResponse :definition-)
      "#____________________________________________________________________________________________
# set_velj_rt
#____________________________________________________________________________________________
float64[6] vel
---
bool       success
")



(provide :dsr_msgs/SetVelJRT "54c890dbfd325b76bab7dbd45237c7df")


