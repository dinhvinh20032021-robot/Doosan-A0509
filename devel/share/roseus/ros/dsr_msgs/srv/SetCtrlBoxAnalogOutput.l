;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetCtrlBoxAnalogOutput)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetCtrlBoxAnalogOutput (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETCTRLBOXANALOGOUTPUT")
  (make-package "DSR_MSGS::SETCTRLBOXANALOGOUTPUT"))
(unless (find-package "DSR_MSGS::SETCTRLBOXANALOGOUTPUTREQUEST")
  (make-package "DSR_MSGS::SETCTRLBOXANALOGOUTPUTREQUEST"))
(unless (find-package "DSR_MSGS::SETCTRLBOXANALOGOUTPUTRESPONSE")
  (make-package "DSR_MSGS::SETCTRLBOXANALOGOUTPUTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetCtrlBoxAnalogOutputRequest
  :super ros::object
  :slots (_channel _value ))

(defmethod dsr_msgs::SetCtrlBoxAnalogOutputRequest
  (:init
   (&key
    ((:channel __channel) 0)
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _channel (round __channel))
   (setq _value (float __value))
   self)
  (:channel
   (&optional __channel)
   (if __channel (setq _channel __channel)) _channel)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; int8 _channel
    1
    ;; float64 _value
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _channel
       (write-byte _channel s)
     ;; float64 _value
       (sys::poke _value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _channel
     (setq _channel (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _channel 127) (setq _channel (- _channel 256)))
   ;; float64 _value
     (setq _value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass dsr_msgs::SetCtrlBoxAnalogOutputResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetCtrlBoxAnalogOutputResponse
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

(defclass dsr_msgs::SetCtrlBoxAnalogOutput
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetCtrlBoxAnalogOutput :md5sum-) "bb0effb74873abe56de0ddcef9d502cb")
(setf (get dsr_msgs::SetCtrlBoxAnalogOutput :datatype-) "dsr_msgs/SetCtrlBoxAnalogOutput")
(setf (get dsr_msgs::SetCtrlBoxAnalogOutput :request) dsr_msgs::SetCtrlBoxAnalogOutputRequest)
(setf (get dsr_msgs::SetCtrlBoxAnalogOutput :response) dsr_msgs::SetCtrlBoxAnalogOutputResponse)

(defmethod dsr_msgs::SetCtrlBoxAnalogOutputRequest
  (:response () (instance dsr_msgs::SetCtrlBoxAnalogOutputResponse :init)))

(setf (get dsr_msgs::SetCtrlBoxAnalogOutputRequest :md5sum-) "bb0effb74873abe56de0ddcef9d502cb")
(setf (get dsr_msgs::SetCtrlBoxAnalogOutputRequest :datatype-) "dsr_msgs/SetCtrlBoxAnalogOutputRequest")
(setf (get dsr_msgs::SetCtrlBoxAnalogOutputRequest :definition-)
      "#____________________________________________________________________________________________
# set_ctrl_box_analog_output  
#____________________________________________________________________________________________

int8        channel  # 1 = ch1, 2= ch2 
float64     value   #
---
bool        success
")

(setf (get dsr_msgs::SetCtrlBoxAnalogOutputResponse :md5sum-) "bb0effb74873abe56de0ddcef9d502cb")
(setf (get dsr_msgs::SetCtrlBoxAnalogOutputResponse :datatype-) "dsr_msgs/SetCtrlBoxAnalogOutputResponse")
(setf (get dsr_msgs::SetCtrlBoxAnalogOutputResponse :definition-)
      "#____________________________________________________________________________________________
# set_ctrl_box_analog_output  
#____________________________________________________________________________________________

int8        channel  # 1 = ch1, 2= ch2 
float64     value   #
---
bool        success
")



(provide :dsr_msgs/SetCtrlBoxAnalogOutput "bb0effb74873abe56de0ddcef9d502cb")


