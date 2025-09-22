;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetCtrlBoxAnalogInput)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetCtrlBoxAnalogInput (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETCTRLBOXANALOGINPUT")
  (make-package "DSR_MSGS::GETCTRLBOXANALOGINPUT"))
(unless (find-package "DSR_MSGS::GETCTRLBOXANALOGINPUTREQUEST")
  (make-package "DSR_MSGS::GETCTRLBOXANALOGINPUTREQUEST"))
(unless (find-package "DSR_MSGS::GETCTRLBOXANALOGINPUTRESPONSE")
  (make-package "DSR_MSGS::GETCTRLBOXANALOGINPUTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetCtrlBoxAnalogInputRequest
  :super ros::object
  :slots (_channel ))

(defmethod dsr_msgs::GetCtrlBoxAnalogInputRequest
  (:init
   (&key
    ((:channel __channel) 0)
    )
   (send-super :init)
   (setq _channel (round __channel))
   self)
  (:channel
   (&optional __channel)
   (if __channel (setq _channel __channel)) _channel)
  (:serialization-length
   ()
   (+
    ;; int8 _channel
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _channel
       (write-byte _channel s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _channel
     (setq _channel (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _channel 127) (setq _channel (- _channel 256)))
   ;;
   self)
  )

(defclass dsr_msgs::GetCtrlBoxAnalogInputResponse
  :super ros::object
  :slots (_value _success ))

(defmethod dsr_msgs::GetCtrlBoxAnalogInputResponse
  (:init
   (&key
    ((:value __value) 0.0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _value (float __value))
   (setq _success __success)
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float64 _value
    8
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _value
       (sys::poke _value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _value
     (setq _value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetCtrlBoxAnalogInput
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetCtrlBoxAnalogInput :md5sum-) "bfe8574a1267a2cf39f4166121f897f2")
(setf (get dsr_msgs::GetCtrlBoxAnalogInput :datatype-) "dsr_msgs/GetCtrlBoxAnalogInput")
(setf (get dsr_msgs::GetCtrlBoxAnalogInput :request) dsr_msgs::GetCtrlBoxAnalogInputRequest)
(setf (get dsr_msgs::GetCtrlBoxAnalogInput :response) dsr_msgs::GetCtrlBoxAnalogInputResponse)

(defmethod dsr_msgs::GetCtrlBoxAnalogInputRequest
  (:response () (instance dsr_msgs::GetCtrlBoxAnalogInputResponse :init)))

(setf (get dsr_msgs::GetCtrlBoxAnalogInputRequest :md5sum-) "bfe8574a1267a2cf39f4166121f897f2")
(setf (get dsr_msgs::GetCtrlBoxAnalogInputRequest :datatype-) "dsr_msgs/GetCtrlBoxAnalogInputRequest")
(setf (get dsr_msgs::GetCtrlBoxAnalogInputRequest :definition-)
      "#____________________________________________________________________________________________
# get_analog_input 
#____________________________________________________________________________________________

int8        channel    # 1 = ch1, 2= ch2
---
float64     value
bool        success
")

(setf (get dsr_msgs::GetCtrlBoxAnalogInputResponse :md5sum-) "bfe8574a1267a2cf39f4166121f897f2")
(setf (get dsr_msgs::GetCtrlBoxAnalogInputResponse :datatype-) "dsr_msgs/GetCtrlBoxAnalogInputResponse")
(setf (get dsr_msgs::GetCtrlBoxAnalogInputResponse :definition-)
      "#____________________________________________________________________________________________
# get_analog_input 
#____________________________________________________________________________________________

int8        channel    # 1 = ch1, 2= ch2
---
float64     value
bool        success
")



(provide :dsr_msgs/GetCtrlBoxAnalogInput "bfe8574a1267a2cf39f4166121f897f2")


