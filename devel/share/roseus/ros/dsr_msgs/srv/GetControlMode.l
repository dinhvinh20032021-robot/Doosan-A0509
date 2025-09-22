;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetControlMode)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetControlMode (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETCONTROLMODE")
  (make-package "DSR_MSGS::GETCONTROLMODE"))
(unless (find-package "DSR_MSGS::GETCONTROLMODEREQUEST")
  (make-package "DSR_MSGS::GETCONTROLMODEREQUEST"))
(unless (find-package "DSR_MSGS::GETCONTROLMODERESPONSE")
  (make-package "DSR_MSGS::GETCONTROLMODERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetControlModeRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetControlModeRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass dsr_msgs::GetControlModeResponse
  :super ros::object
  :slots (_control_mode _success ))

(defmethod dsr_msgs::GetControlModeResponse
  (:init
   (&key
    ((:control_mode __control_mode) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _control_mode (round __control_mode))
   (setq _success __success)
   self)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int8 _control_mode
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _control_mode
       (write-byte _control_mode s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _control_mode
     (setq _control_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _control_mode 127) (setq _control_mode (- _control_mode 256)))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetControlMode
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetControlMode :md5sum-) "bc4e735fc2aafec8676fb72eaa5429bc")
(setf (get dsr_msgs::GetControlMode :datatype-) "dsr_msgs/GetControlMode")
(setf (get dsr_msgs::GetControlMode :request) dsr_msgs::GetControlModeRequest)
(setf (get dsr_msgs::GetControlMode :response) dsr_msgs::GetControlModeResponse)

(defmethod dsr_msgs::GetControlModeRequest
  (:response () (instance dsr_msgs::GetControlModeResponse :init)))

(setf (get dsr_msgs::GetControlModeRequest :md5sum-) "bc4e735fc2aafec8676fb72eaa5429bc")
(setf (get dsr_msgs::GetControlModeRequest :datatype-) "dsr_msgs/GetControlModeRequest")
(setf (get dsr_msgs::GetControlModeRequest :definition-)
      "#____________________________________________________________________________________________
# get_control_mode()  
#____________________________________________________________________________________________
# This service returns the current control mode.

---
int8    control_mode        # Control mode : Position control mode(3), Torque control mode(4)
bool    success 

")

(setf (get dsr_msgs::GetControlModeResponse :md5sum-) "bc4e735fc2aafec8676fb72eaa5429bc")
(setf (get dsr_msgs::GetControlModeResponse :datatype-) "dsr_msgs/GetControlModeResponse")
(setf (get dsr_msgs::GetControlModeResponse :definition-)
      "#____________________________________________________________________________________________
# get_control_mode()  
#____________________________________________________________________________________________
# This service returns the current control mode.

---
int8    control_mode        # Control mode : Position control mode(3), Torque control mode(4)
bool    success 

")



(provide :dsr_msgs/GetControlMode "bc4e735fc2aafec8676fb72eaa5429bc")


