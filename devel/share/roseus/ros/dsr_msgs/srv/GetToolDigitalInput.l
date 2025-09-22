;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetToolDigitalInput)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetToolDigitalInput (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETTOOLDIGITALINPUT")
  (make-package "DSR_MSGS::GETTOOLDIGITALINPUT"))
(unless (find-package "DSR_MSGS::GETTOOLDIGITALINPUTREQUEST")
  (make-package "DSR_MSGS::GETTOOLDIGITALINPUTREQUEST"))
(unless (find-package "DSR_MSGS::GETTOOLDIGITALINPUTRESPONSE")
  (make-package "DSR_MSGS::GETTOOLDIGITALINPUTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetToolDigitalInputRequest
  :super ros::object
  :slots (_index ))

(defmethod dsr_msgs::GetToolDigitalInputRequest
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

(defclass dsr_msgs::GetToolDigitalInputResponse
  :super ros::object
  :slots (_value _success ))

(defmethod dsr_msgs::GetToolDigitalInputResponse
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

(defclass dsr_msgs::GetToolDigitalInput
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetToolDigitalInput :md5sum-) "66249e10e881902f2908d5046d836788")
(setf (get dsr_msgs::GetToolDigitalInput :datatype-) "dsr_msgs/GetToolDigitalInput")
(setf (get dsr_msgs::GetToolDigitalInput :request) dsr_msgs::GetToolDigitalInputRequest)
(setf (get dsr_msgs::GetToolDigitalInput :response) dsr_msgs::GetToolDigitalInputResponse)

(defmethod dsr_msgs::GetToolDigitalInputRequest
  (:response () (instance dsr_msgs::GetToolDigitalInputResponse :init)))

(setf (get dsr_msgs::GetToolDigitalInputRequest :md5sum-) "66249e10e881902f2908d5046d836788")
(setf (get dsr_msgs::GetToolDigitalInputRequest :datatype-) "dsr_msgs/GetToolDigitalInputRequest")
(setf (get dsr_msgs::GetToolDigitalInputRequest :definition-)
      "#____________________________________________________________________________________________
# get_tool_digital_input  
# This service gets the current controlbox io output status.
#____________________________________________________________________________________________

int8        index    # Digital Input in Flange(1 ~ 6) 
                     # <GPIO_TOOL_DIGITAL_INDEX> 
---
int8        value    # 0=OFF, 1=ON
bool        success
")

(setf (get dsr_msgs::GetToolDigitalInputResponse :md5sum-) "66249e10e881902f2908d5046d836788")
(setf (get dsr_msgs::GetToolDigitalInputResponse :datatype-) "dsr_msgs/GetToolDigitalInputResponse")
(setf (get dsr_msgs::GetToolDigitalInputResponse :definition-)
      "#____________________________________________________________________________________________
# get_tool_digital_input  
# This service gets the current controlbox io output status.
#____________________________________________________________________________________________

int8        index    # Digital Input in Flange(1 ~ 6) 
                     # <GPIO_TOOL_DIGITAL_INDEX> 
---
int8        value    # 0=OFF, 1=ON
bool        success
")



(provide :dsr_msgs/GetToolDigitalInput "66249e10e881902f2908d5046d836788")


