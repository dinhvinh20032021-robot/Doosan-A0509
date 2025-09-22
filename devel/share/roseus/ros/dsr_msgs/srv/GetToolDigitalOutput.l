;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetToolDigitalOutput)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetToolDigitalOutput (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETTOOLDIGITALOUTPUT")
  (make-package "DSR_MSGS::GETTOOLDIGITALOUTPUT"))
(unless (find-package "DSR_MSGS::GETTOOLDIGITALOUTPUTREQUEST")
  (make-package "DSR_MSGS::GETTOOLDIGITALOUTPUTREQUEST"))
(unless (find-package "DSR_MSGS::GETTOOLDIGITALOUTPUTRESPONSE")
  (make-package "DSR_MSGS::GETTOOLDIGITALOUTPUTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetToolDigitalOutputRequest
  :super ros::object
  :slots (_index ))

(defmethod dsr_msgs::GetToolDigitalOutputRequest
  (:init
   (&key
    ((:index __index) 0)
    )
   (send-super :init)
   (setq _index (round __index))
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:serialization-length
   ()
   (+
    ;; int8 _index
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _index
       (write-byte _index s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _index
     (setq _index (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _index 127) (setq _index (- _index 256)))
   ;;
   self)
  )

(defclass dsr_msgs::GetToolDigitalOutputResponse
  :super ros::object
  :slots (_value _success ))

(defmethod dsr_msgs::GetToolDigitalOutputResponse
  (:init
   (&key
    ((:value __value) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _value (round __value))
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
    ;; int8 _value
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _value
       (write-byte _value s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _value
     (setq _value (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _value 127) (setq _value (- _value 256)))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetToolDigitalOutput
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetToolDigitalOutput :md5sum-) "66249e10e881902f2908d5046d836788")
(setf (get dsr_msgs::GetToolDigitalOutput :datatype-) "dsr_msgs/GetToolDigitalOutput")
(setf (get dsr_msgs::GetToolDigitalOutput :request) dsr_msgs::GetToolDigitalOutputRequest)
(setf (get dsr_msgs::GetToolDigitalOutput :response) dsr_msgs::GetToolDigitalOutputResponse)

(defmethod dsr_msgs::GetToolDigitalOutputRequest
  (:response () (instance dsr_msgs::GetToolDigitalOutputResponse :init)))

(setf (get dsr_msgs::GetToolDigitalOutputRequest :md5sum-) "66249e10e881902f2908d5046d836788")
(setf (get dsr_msgs::GetToolDigitalOutputRequest :datatype-) "dsr_msgs/GetToolDigitalOutputRequest")
(setf (get dsr_msgs::GetToolDigitalOutputRequest :definition-)
      "#____________________________________________________________________________________________
# get_tool_digital_output  
# This service gets the current tool io output status.
#____________________________________________________________________________________________

int8       index   # flange digital output port(1 ~ 6)
---
int8       value   # Current output status (0 : ON, 1 : OFF)
bool       success
")

(setf (get dsr_msgs::GetToolDigitalOutputResponse :md5sum-) "66249e10e881902f2908d5046d836788")
(setf (get dsr_msgs::GetToolDigitalOutputResponse :datatype-) "dsr_msgs/GetToolDigitalOutputResponse")
(setf (get dsr_msgs::GetToolDigitalOutputResponse :definition-)
      "#____________________________________________________________________________________________
# get_tool_digital_output  
# This service gets the current tool io output status.
#____________________________________________________________________________________________

int8       index   # flange digital output port(1 ~ 6)
---
int8       value   # Current output status (0 : ON, 1 : OFF)
bool       success
")



(provide :dsr_msgs/GetToolDigitalOutput "66249e10e881902f2908d5046d836788")


