;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetRTControlInput)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetRTControlInput (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETRTCONTROLINPUT")
  (make-package "DSR_MSGS::SETRTCONTROLINPUT"))
(unless (find-package "DSR_MSGS::SETRTCONTROLINPUTREQUEST")
  (make-package "DSR_MSGS::SETRTCONTROLINPUTREQUEST"))
(unless (find-package "DSR_MSGS::SETRTCONTROLINPUTRESPONSE")
  (make-package "DSR_MSGS::SETRTCONTROLINPUTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetRTControlInputRequest
  :super ros::object
  :slots (_version _period _loss ))

(defmethod dsr_msgs::SetRTControlInputRequest
  (:init
   (&key
    ((:version __version) "")
    ((:period __period) 0.0)
    ((:loss __loss) 0)
    )
   (send-super :init)
   (setq _version (string __version))
   (setq _period (float __period))
   (setq _loss (round __loss))
   self)
  (:version
   (&optional __version)
   (if __version (setq _version __version)) _version)
  (:period
   (&optional __period)
   (if __period (setq _period __period)) _period)
  (:loss
   (&optional __loss)
   (if __loss (setq _loss __loss)) _loss)
  (:serialization-length
   ()
   (+
    ;; string _version
    4 (length _version)
    ;; float64 _period
    8
    ;; int32 _loss
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _version
       (write-long (length _version) s) (princ _version s)
     ;; float64 _period
       (sys::poke _period (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _loss
       (write-long _loss s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _version
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _version (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _period
     (setq _period (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _loss
     (setq _loss (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dsr_msgs::SetRTControlInputResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetRTControlInputResponse
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

(defclass dsr_msgs::SetRTControlInput
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetRTControlInput :md5sum-) "adc3c45016368698475e43bea709f99f")
(setf (get dsr_msgs::SetRTControlInput :datatype-) "dsr_msgs/SetRTControlInput")
(setf (get dsr_msgs::SetRTControlInput :request) dsr_msgs::SetRTControlInputRequest)
(setf (get dsr_msgs::SetRTControlInput :response) dsr_msgs::SetRTControlInputResponse)

(defmethod dsr_msgs::SetRTControlInputRequest
  (:response () (instance dsr_msgs::SetRTControlInputResponse :init)))

(setf (get dsr_msgs::SetRTControlInputRequest :md5sum-) "adc3c45016368698475e43bea709f99f")
(setf (get dsr_msgs::SetRTControlInputRequest :datatype-) "dsr_msgs/SetRTControlInputRequest")
(setf (get dsr_msgs::SetRTControlInputRequest :definition-)
      "#____________________________________________________________________________________________
# set_rt_control_input
#____________________________________________________________________________________________
string     version
float64    period
int32      loss
---
bool       success
")

(setf (get dsr_msgs::SetRTControlInputResponse :md5sum-) "adc3c45016368698475e43bea709f99f")
(setf (get dsr_msgs::SetRTControlInputResponse :datatype-) "dsr_msgs/SetRTControlInputResponse")
(setf (get dsr_msgs::SetRTControlInputResponse :definition-)
      "#____________________________________________________________________________________________
# set_rt_control_input
#____________________________________________________________________________________________
string     version
float64    period
int32      loss
---
bool       success
")



(provide :dsr_msgs/SetRTControlInput "adc3c45016368698475e43bea709f99f")


