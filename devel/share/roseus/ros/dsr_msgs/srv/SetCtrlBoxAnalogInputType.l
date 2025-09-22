;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetCtrlBoxAnalogInputType)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetCtrlBoxAnalogInputType (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETCTRLBOXANALOGINPUTTYPE")
  (make-package "DSR_MSGS::SETCTRLBOXANALOGINPUTTYPE"))
(unless (find-package "DSR_MSGS::SETCTRLBOXANALOGINPUTTYPEREQUEST")
  (make-package "DSR_MSGS::SETCTRLBOXANALOGINPUTTYPEREQUEST"))
(unless (find-package "DSR_MSGS::SETCTRLBOXANALOGINPUTTYPERESPONSE")
  (make-package "DSR_MSGS::SETCTRLBOXANALOGINPUTTYPERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetCtrlBoxAnalogInputTypeRequest
  :super ros::object
  :slots (_channel _mode ))

(defmethod dsr_msgs::SetCtrlBoxAnalogInputTypeRequest
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

(defclass dsr_msgs::SetCtrlBoxAnalogInputTypeResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetCtrlBoxAnalogInputTypeResponse
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

(defclass dsr_msgs::SetCtrlBoxAnalogInputType
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetCtrlBoxAnalogInputType :md5sum-) "edb11f75523beab953b2183675ee124d")
(setf (get dsr_msgs::SetCtrlBoxAnalogInputType :datatype-) "dsr_msgs/SetCtrlBoxAnalogInputType")
(setf (get dsr_msgs::SetCtrlBoxAnalogInputType :request) dsr_msgs::SetCtrlBoxAnalogInputTypeRequest)
(setf (get dsr_msgs::SetCtrlBoxAnalogInputType :response) dsr_msgs::SetCtrlBoxAnalogInputTypeResponse)

(defmethod dsr_msgs::SetCtrlBoxAnalogInputTypeRequest
  (:response () (instance dsr_msgs::SetCtrlBoxAnalogInputTypeResponse :init)))

(setf (get dsr_msgs::SetCtrlBoxAnalogInputTypeRequest :md5sum-) "edb11f75523beab953b2183675ee124d")
(setf (get dsr_msgs::SetCtrlBoxAnalogInputTypeRequest :datatype-) "dsr_msgs/SetCtrlBoxAnalogInputTypeRequest")
(setf (get dsr_msgs::SetCtrlBoxAnalogInputTypeRequest :definition-)
      "#____________________________________________________________________________________________
# set_ctrl_box_analog_input_type  
#____________________________________________________________________________________________

int8        channel  # 1 = ch1, 2= ch2 
int8        mode     # 0 = current, 1 = voltage
---
bool        success
")

(setf (get dsr_msgs::SetCtrlBoxAnalogInputTypeResponse :md5sum-) "edb11f75523beab953b2183675ee124d")
(setf (get dsr_msgs::SetCtrlBoxAnalogInputTypeResponse :datatype-) "dsr_msgs/SetCtrlBoxAnalogInputTypeResponse")
(setf (get dsr_msgs::SetCtrlBoxAnalogInputTypeResponse :definition-)
      "#____________________________________________________________________________________________
# set_ctrl_box_analog_input_type  
#____________________________________________________________________________________________

int8        channel  # 1 = ch1, 2= ch2 
int8        mode     # 0 = current, 1 = voltage
---
bool        success
")



(provide :dsr_msgs/SetCtrlBoxAnalogInputType "edb11f75523beab953b2183675ee124d")


