;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetCtrlBoxDigitalInput)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetCtrlBoxDigitalInput (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETCTRLBOXDIGITALINPUT")
  (make-package "DSR_MSGS::GETCTRLBOXDIGITALINPUT"))
(unless (find-package "DSR_MSGS::GETCTRLBOXDIGITALINPUTREQUEST")
  (make-package "DSR_MSGS::GETCTRLBOXDIGITALINPUTREQUEST"))
(unless (find-package "DSR_MSGS::GETCTRLBOXDIGITALINPUTRESPONSE")
  (make-package "DSR_MSGS::GETCTRLBOXDIGITALINPUTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetCtrlBoxDigitalInputRequest
  :super ros::object
  :slots (_index ))

(defmethod dsr_msgs::GetCtrlBoxDigitalInputRequest
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

(defclass dsr_msgs::GetCtrlBoxDigitalInputResponse
  :super ros::object
  :slots (_value _success ))

(defmethod dsr_msgs::GetCtrlBoxDigitalInputResponse
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

(defclass dsr_msgs::GetCtrlBoxDigitalInput
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetCtrlBoxDigitalInput :md5sum-) "66249e10e881902f2908d5046d836788")
(setf (get dsr_msgs::GetCtrlBoxDigitalInput :datatype-) "dsr_msgs/GetCtrlBoxDigitalInput")
(setf (get dsr_msgs::GetCtrlBoxDigitalInput :request) dsr_msgs::GetCtrlBoxDigitalInputRequest)
(setf (get dsr_msgs::GetCtrlBoxDigitalInput :response) dsr_msgs::GetCtrlBoxDigitalInputResponse)

(defmethod dsr_msgs::GetCtrlBoxDigitalInputRequest
  (:response () (instance dsr_msgs::GetCtrlBoxDigitalInputResponse :init)))

(setf (get dsr_msgs::GetCtrlBoxDigitalInputRequest :md5sum-) "66249e10e881902f2908d5046d836788")
(setf (get dsr_msgs::GetCtrlBoxDigitalInputRequest :datatype-) "dsr_msgs/GetCtrlBoxDigitalInputRequest")
(setf (get dsr_msgs::GetCtrlBoxDigitalInputRequest :definition-)
      "#____________________________________________________________________________________________
# get_digital_input
# This service reads the I/O signals from digital contact points of the controller and reads the digital input contact value.
#____________________________________________________________________________________________

int8        index    # Digital Input in Control Box(1 ~ 16) 
                     # <GPIO_CTRLBOX_DIGITAL_INDEX>
---
int8        value    # 0=OFF, 1=ON
bool        success
")

(setf (get dsr_msgs::GetCtrlBoxDigitalInputResponse :md5sum-) "66249e10e881902f2908d5046d836788")
(setf (get dsr_msgs::GetCtrlBoxDigitalInputResponse :datatype-) "dsr_msgs/GetCtrlBoxDigitalInputResponse")
(setf (get dsr_msgs::GetCtrlBoxDigitalInputResponse :definition-)
      "#____________________________________________________________________________________________
# get_digital_input
# This service reads the I/O signals from digital contact points of the controller and reads the digital input contact value.
#____________________________________________________________________________________________

int8        index    # Digital Input in Control Box(1 ~ 16) 
                     # <GPIO_CTRLBOX_DIGITAL_INDEX>
---
int8        value    # 0=OFF, 1=ON
bool        success
")



(provide :dsr_msgs/GetCtrlBoxDigitalInput "66249e10e881902f2908d5046d836788")


