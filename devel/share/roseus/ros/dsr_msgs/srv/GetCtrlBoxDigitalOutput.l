;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetCtrlBoxDigitalOutput)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetCtrlBoxDigitalOutput (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETCTRLBOXDIGITALOUTPUT")
  (make-package "DSR_MSGS::GETCTRLBOXDIGITALOUTPUT"))
(unless (find-package "DSR_MSGS::GETCTRLBOXDIGITALOUTPUTREQUEST")
  (make-package "DSR_MSGS::GETCTRLBOXDIGITALOUTPUTREQUEST"))
(unless (find-package "DSR_MSGS::GETCTRLBOXDIGITALOUTPUTRESPONSE")
  (make-package "DSR_MSGS::GETCTRLBOXDIGITALOUTPUTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetCtrlBoxDigitalOutputRequest
  :super ros::object
  :slots (_index ))

(defmethod dsr_msgs::GetCtrlBoxDigitalOutputRequest
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

(defclass dsr_msgs::GetCtrlBoxDigitalOutputResponse
  :super ros::object
  :slots (_value _success ))

(defmethod dsr_msgs::GetCtrlBoxDigitalOutputResponse
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

(defclass dsr_msgs::GetCtrlBoxDigitalOutput
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetCtrlBoxDigitalOutput :md5sum-) "66249e10e881902f2908d5046d836788")
(setf (get dsr_msgs::GetCtrlBoxDigitalOutput :datatype-) "dsr_msgs/GetCtrlBoxDigitalOutput")
(setf (get dsr_msgs::GetCtrlBoxDigitalOutput :request) dsr_msgs::GetCtrlBoxDigitalOutputRequest)
(setf (get dsr_msgs::GetCtrlBoxDigitalOutput :response) dsr_msgs::GetCtrlBoxDigitalOutputResponse)

(defmethod dsr_msgs::GetCtrlBoxDigitalOutputRequest
  (:response () (instance dsr_msgs::GetCtrlBoxDigitalOutputResponse :init)))

(setf (get dsr_msgs::GetCtrlBoxDigitalOutputRequest :md5sum-) "66249e10e881902f2908d5046d836788")
(setf (get dsr_msgs::GetCtrlBoxDigitalOutputRequest :datatype-) "dsr_msgs/GetCtrlBoxDigitalOutputRequest")
(setf (get dsr_msgs::GetCtrlBoxDigitalOutputRequest :definition-)
      "#____________________________________________________________________________________________
# get_digital_output  
#____________________________________________________________________________________________

int8       index    # ctrlbox digital output port(1 ~ 16)
---
int8       value    # Current output status (0 : ON, 1 : OFF)
bool       success

")

(setf (get dsr_msgs::GetCtrlBoxDigitalOutputResponse :md5sum-) "66249e10e881902f2908d5046d836788")
(setf (get dsr_msgs::GetCtrlBoxDigitalOutputResponse :datatype-) "dsr_msgs/GetCtrlBoxDigitalOutputResponse")
(setf (get dsr_msgs::GetCtrlBoxDigitalOutputResponse :definition-)
      "#____________________________________________________________________________________________
# get_digital_output  
#____________________________________________________________________________________________

int8       index    # ctrlbox digital output port(1 ~ 16)
---
int8       value    # Current output status (0 : ON, 1 : OFF)
bool       success

")



(provide :dsr_msgs/GetCtrlBoxDigitalOutput "66249e10e881902f2908d5046d836788")


