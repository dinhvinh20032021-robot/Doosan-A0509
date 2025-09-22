;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetCtrlBoxAnalogOutputType)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetCtrlBoxAnalogOutputType (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETCTRLBOXANALOGOUTPUTTYPE")
  (make-package "DSR_MSGS::SETCTRLBOXANALOGOUTPUTTYPE"))
(unless (find-package "DSR_MSGS::SETCTRLBOXANALOGOUTPUTTYPEREQUEST")
  (make-package "DSR_MSGS::SETCTRLBOXANALOGOUTPUTTYPEREQUEST"))
(unless (find-package "DSR_MSGS::SETCTRLBOXANALOGOUTPUTTYPERESPONSE")
  (make-package "DSR_MSGS::SETCTRLBOXANALOGOUTPUTTYPERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetCtrlBoxAnalogOutputTypeRequest
  :super ros::object
  :slots (_channel _mode ))

(defmethod dsr_msgs::SetCtrlBoxAnalogOutputTypeRequest
  (:init
   (&key
    ((:channel __channel) 0)
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _channel (round __channel))
   (setq _mode (round __mode))
   self)
  (:channel
   (&optional __channel)
   (if __channel (setq _channel __channel)) _channel)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; int8 _channel
    1
    ;; int8 _mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _channel
       (write-byte _channel s)
     ;; int8 _mode
       (write-byte _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _channel
     (setq _channel (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _channel 127) (setq _channel (- _channel 256)))
   ;; int8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;;
   self)
  )

(defclass dsr_msgs::SetCtrlBoxAnalogOutputTypeResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetCtrlBoxAnalogOutputTypeResponse
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

(defclass dsr_msgs::SetCtrlBoxAnalogOutputType
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetCtrlBoxAnalogOutputType :md5sum-) "edb11f75523beab953b2183675ee124d")
(setf (get dsr_msgs::SetCtrlBoxAnalogOutputType :datatype-) "dsr_msgs/SetCtrlBoxAnalogOutputType")
(setf (get dsr_msgs::SetCtrlBoxAnalogOutputType :request) dsr_msgs::SetCtrlBoxAnalogOutputTypeRequest)
(setf (get dsr_msgs::SetCtrlBoxAnalogOutputType :response) dsr_msgs::SetCtrlBoxAnalogOutputTypeResponse)

(defmethod dsr_msgs::SetCtrlBoxAnalogOutputTypeRequest
  (:response () (instance dsr_msgs::SetCtrlBoxAnalogOutputTypeResponse :init)))

(setf (get dsr_msgs::SetCtrlBoxAnalogOutputTypeRequest :md5sum-) "edb11f75523beab953b2183675ee124d")
(setf (get dsr_msgs::SetCtrlBoxAnalogOutputTypeRequest :datatype-) "dsr_msgs/SetCtrlBoxAnalogOutputTypeRequest")
(setf (get dsr_msgs::SetCtrlBoxAnalogOutputTypeRequest :definition-)
      "#____________________________________________________________________________________________
# set_ctrl_box_analog_output_type  
#____________________________________________________________________________________________

int8        channel  # 1 = ch1, 2= ch2 
int8        mode     # 0 = current, 1 = voltage
---
bool        success
")

(setf (get dsr_msgs::SetCtrlBoxAnalogOutputTypeResponse :md5sum-) "edb11f75523beab953b2183675ee124d")
(setf (get dsr_msgs::SetCtrlBoxAnalogOutputTypeResponse :datatype-) "dsr_msgs/SetCtrlBoxAnalogOutputTypeResponse")
(setf (get dsr_msgs::SetCtrlBoxAnalogOutputTypeResponse :definition-)
      "#____________________________________________________________________________________________
# set_ctrl_box_analog_output_type  
#____________________________________________________________________________________________

int8        channel  # 1 = ch1, 2= ch2 
int8        mode     # 0 = current, 1 = voltage
---
bool        success
")



(provide :dsr_msgs/SetCtrlBoxAnalogOutputType "edb11f75523beab953b2183675ee124d")


