;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetCtrlBoxDigitalOutput)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetCtrlBoxDigitalOutput (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETCTRLBOXDIGITALOUTPUT")
  (make-package "DSR_MSGS::SETCTRLBOXDIGITALOUTPUT"))
(unless (find-package "DSR_MSGS::SETCTRLBOXDIGITALOUTPUTREQUEST")
  (make-package "DSR_MSGS::SETCTRLBOXDIGITALOUTPUTREQUEST"))
(unless (find-package "DSR_MSGS::SETCTRLBOXDIGITALOUTPUTRESPONSE")
  (make-package "DSR_MSGS::SETCTRLBOXDIGITALOUTPUTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetCtrlBoxDigitalOutputRequest
  :super ros::object
  :slots (_index _value ))

(defmethod dsr_msgs::SetCtrlBoxDigitalOutputRequest
  (:init
   (&key
    ((:index __index) 0)
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _value (round __value))
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; int8 _index
    1
    ;; int8 _value
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _index
       (write-byte _index s)
     ;; int8 _value
       (write-byte _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _index
     (setq _index (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _index 127) (setq _index (- _index 256)))
   ;; int8 _value
     (setq _value (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _value 127) (setq _value (- _value 256)))
   ;;
   self)
  )

(defclass dsr_msgs::SetCtrlBoxDigitalOutputResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetCtrlBoxDigitalOutputResponse
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

(defclass dsr_msgs::SetCtrlBoxDigitalOutput
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetCtrlBoxDigitalOutput :md5sum-) "fb5dce2e6988e39a422dd00d9d97c412")
(setf (get dsr_msgs::SetCtrlBoxDigitalOutput :datatype-) "dsr_msgs/SetCtrlBoxDigitalOutput")
(setf (get dsr_msgs::SetCtrlBoxDigitalOutput :request) dsr_msgs::SetCtrlBoxDigitalOutputRequest)
(setf (get dsr_msgs::SetCtrlBoxDigitalOutput :response) dsr_msgs::SetCtrlBoxDigitalOutputResponse)

(defmethod dsr_msgs::SetCtrlBoxDigitalOutputRequest
  (:response () (instance dsr_msgs::SetCtrlBoxDigitalOutputResponse :init)))

(setf (get dsr_msgs::SetCtrlBoxDigitalOutputRequest :md5sum-) "fb5dce2e6988e39a422dd00d9d97c412")
(setf (get dsr_msgs::SetCtrlBoxDigitalOutputRequest :datatype-) "dsr_msgs/SetCtrlBoxDigitalOutputRequest")
(setf (get dsr_msgs::SetCtrlBoxDigitalOutputRequest :definition-)
      "#____________________________________________________________________________________________
# set_digital_output  
#____________________________________________________________________________________________

int8       index    # ctrlbox digital output port(1 ~ 16)
int8       value    # 0 : ON, 1 : OFF
---
bool        success

")

(setf (get dsr_msgs::SetCtrlBoxDigitalOutputResponse :md5sum-) "fb5dce2e6988e39a422dd00d9d97c412")
(setf (get dsr_msgs::SetCtrlBoxDigitalOutputResponse :datatype-) "dsr_msgs/SetCtrlBoxDigitalOutputResponse")
(setf (get dsr_msgs::SetCtrlBoxDigitalOutputResponse :definition-)
      "#____________________________________________________________________________________________
# set_digital_output  
#____________________________________________________________________________________________

int8       index    # ctrlbox digital output port(1 ~ 16)
int8       value    # 0 : ON, 1 : OFF
---
bool        success

")



(provide :dsr_msgs/SetCtrlBoxDigitalOutput "fb5dce2e6988e39a422dd00d9d97c412")


