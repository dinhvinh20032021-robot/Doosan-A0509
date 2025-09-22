;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetAccJRT)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetAccJRT (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETACCJRT")
  (make-package "DSR_MSGS::SETACCJRT"))
(unless (find-package "DSR_MSGS::SETACCJRTREQUEST")
  (make-package "DSR_MSGS::SETACCJRTREQUEST"))
(unless (find-package "DSR_MSGS::SETACCJRTRESPONSE")
  (make-package "DSR_MSGS::SETACCJRTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetAccJRTRequest
  :super ros::object
  :slots (_acc ))

(defmethod dsr_msgs::SetAccJRTRequest
  (:init
   (&key
    ((:acc __acc) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _acc __acc)
   self)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:serialization-length
   ()
   (+
    ;; float64[6] _acc
    (* 8    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _acc
     (dotimes (i 6)
       (sys::poke (elt _acc i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _acc
   (dotimes (i (length _acc))
     (setf (elt _acc i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass dsr_msgs::SetAccJRTResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetAccJRTResponse
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

(defclass dsr_msgs::SetAccJRT
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetAccJRT :md5sum-) "813f535e4d5aa4d44e3a153e7c2700e3")
(setf (get dsr_msgs::SetAccJRT :datatype-) "dsr_msgs/SetAccJRT")
(setf (get dsr_msgs::SetAccJRT :request) dsr_msgs::SetAccJRTRequest)
(setf (get dsr_msgs::SetAccJRT :response) dsr_msgs::SetAccJRTResponse)

(defmethod dsr_msgs::SetAccJRTRequest
  (:response () (instance dsr_msgs::SetAccJRTResponse :init)))

(setf (get dsr_msgs::SetAccJRTRequest :md5sum-) "813f535e4d5aa4d44e3a153e7c2700e3")
(setf (get dsr_msgs::SetAccJRTRequest :datatype-) "dsr_msgs/SetAccJRTRequest")
(setf (get dsr_msgs::SetAccJRTRequest :definition-)
      "#____________________________________________________________________________________________
# set_accj_rt
#____________________________________________________________________________________________
float64[6] acc
---
bool       success
")

(setf (get dsr_msgs::SetAccJRTResponse :md5sum-) "813f535e4d5aa4d44e3a153e7c2700e3")
(setf (get dsr_msgs::SetAccJRTResponse :datatype-) "dsr_msgs/SetAccJRTResponse")
(setf (get dsr_msgs::SetAccJRTResponse :definition-)
      "#____________________________________________________________________________________________
# set_accj_rt
#____________________________________________________________________________________________
float64[6] acc
---
bool       success
")



(provide :dsr_msgs/SetAccJRT "813f535e4d5aa4d44e3a153e7c2700e3")


