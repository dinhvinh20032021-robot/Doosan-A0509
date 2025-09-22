;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetRTControlOutput)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetRTControlOutput (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETRTCONTROLOUTPUT")
  (make-package "DSR_MSGS::SETRTCONTROLOUTPUT"))
(unless (find-package "DSR_MSGS::SETRTCONTROLOUTPUTREQUEST")
  (make-package "DSR_MSGS::SETRTCONTROLOUTPUTREQUEST"))
(unless (find-package "DSR_MSGS::SETRTCONTROLOUTPUTRESPONSE")
  (make-package "DSR_MSGS::SETRTCONTROLOUTPUTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetRTControlOutputRequest
  :super ros::object
  :slots (_version _period _loss ))

(defmethod dsr_msgs::SetRTControlOutputRequest
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

(defclass dsr_msgs::SetRTControlOutputResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetRTControlOutputResponse
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

(defclass dsr_msgs::SetRTControlOutput
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetRTControlOutput :md5sum-) "adc3c45016368698475e43bea709f99f")
(setf (get dsr_msgs::SetRTControlOutput :datatype-) "dsr_msgs/SetRTControlOutput")
(setf (get dsr_msgs::SetRTControlOutput :request) dsr_msgs::SetRTControlOutputRequest)
(setf (get dsr_msgs::SetRTControlOutput :response) dsr_msgs::SetRTControlOutputResponse)

(defmethod dsr_msgs::SetRTControlOutputRequest
  (:response () (instance dsr_msgs::SetRTControlOutputResponse :init)))

(setf (get dsr_msgs::SetRTControlOutputRequest :md5sum-) "adc3c45016368698475e43bea709f99f")
(setf (get dsr_msgs::SetRTControlOutputRequest :datatype-) "dsr_msgs/SetRTControlOutputRequest")
(setf (get dsr_msgs::SetRTControlOutputRequest :definition-)
      "#____________________________________________________________________________________________
# set_rt_control_output
#____________________________________________________________________________________________
string     version
float64    period
int32      loss
---
bool       success
")

(setf (get dsr_msgs::SetRTControlOutputResponse :md5sum-) "adc3c45016368698475e43bea709f99f")
(setf (get dsr_msgs::SetRTControlOutputResponse :datatype-) "dsr_msgs/SetRTControlOutputResponse")
(setf (get dsr_msgs::SetRTControlOutputResponse :definition-)
      "#____________________________________________________________________________________________
# set_rt_control_output
#____________________________________________________________________________________________
string     version
float64    period
int32      loss
---
bool       success
")



(provide :dsr_msgs/SetRTControlOutput "adc3c45016368698475e43bea709f99f")


