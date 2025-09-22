;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetSafeStopResetType)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetSafeStopResetType (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETSAFESTOPRESETTYPE")
  (make-package "DSR_MSGS::SETSAFESTOPRESETTYPE"))
(unless (find-package "DSR_MSGS::SETSAFESTOPRESETTYPEREQUEST")
  (make-package "DSR_MSGS::SETSAFESTOPRESETTYPEREQUEST"))
(unless (find-package "DSR_MSGS::SETSAFESTOPRESETTYPERESPONSE")
  (make-package "DSR_MSGS::SETSAFESTOPRESETTYPERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetSafeStopResetTypeRequest
  :super ros::object
  :slots (_reset_type ))

(defmethod dsr_msgs::SetSafeStopResetTypeRequest
  (:init
   (&key
    ((:reset_type __reset_type) 0)
    )
   (send-super :init)
   (setq _reset_type (round __reset_type))
   self)
  (:reset_type
   (&optional __reset_type)
   (if __reset_type (setq _reset_type __reset_type)) _reset_type)
  (:serialization-length
   ()
   (+
    ;; int8 _reset_type
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _reset_type
       (write-byte _reset_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _reset_type
     (setq _reset_type (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _reset_type 127) (setq _reset_type (- _reset_type 256)))
   ;;
   self)
  )

(defclass dsr_msgs::SetSafeStopResetTypeResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetSafeStopResetTypeResponse
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

(defclass dsr_msgs::SetSafeStopResetType
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetSafeStopResetType :md5sum-) "956a2bdbbc2e1317a58568aa311b17b7")
(setf (get dsr_msgs::SetSafeStopResetType :datatype-) "dsr_msgs/SetSafeStopResetType")
(setf (get dsr_msgs::SetSafeStopResetType :request) dsr_msgs::SetSafeStopResetTypeRequest)
(setf (get dsr_msgs::SetSafeStopResetType :response) dsr_msgs::SetSafeStopResetTypeResponse)

(defmethod dsr_msgs::SetSafeStopResetTypeRequest
  (:response () (instance dsr_msgs::SetSafeStopResetTypeResponse :init)))

(setf (get dsr_msgs::SetSafeStopResetTypeRequest :md5sum-) "956a2bdbbc2e1317a58568aa311b17b7")
(setf (get dsr_msgs::SetSafeStopResetTypeRequest :datatype-) "dsr_msgs/SetSafeStopResetTypeRequest")
(setf (get dsr_msgs::SetSafeStopResetTypeRequest :definition-)
      "#____________________________________________________________________________________________
# set_safe_stop_reset_type
#____________________________________________________________________________________________

int8 reset_type     # 0=SAFE_STOP_RESET_TYPE_DEFAULT = SAFE_STOP_RESET_TYPE_PROGRAM_STOP , 1= SAFE_STOP_RESET_TYPE_PROGRAM_RESUME 
---
bool success
")

(setf (get dsr_msgs::SetSafeStopResetTypeResponse :md5sum-) "956a2bdbbc2e1317a58568aa311b17b7")
(setf (get dsr_msgs::SetSafeStopResetTypeResponse :datatype-) "dsr_msgs/SetSafeStopResetTypeResponse")
(setf (get dsr_msgs::SetSafeStopResetTypeResponse :definition-)
      "#____________________________________________________________________________________________
# set_safe_stop_reset_type
#____________________________________________________________________________________________

int8 reset_type     # 0=SAFE_STOP_RESET_TYPE_DEFAULT = SAFE_STOP_RESET_TYPE_PROGRAM_STOP , 1= SAFE_STOP_RESET_TYPE_PROGRAM_RESUME 
---
bool success
")



(provide :dsr_msgs/SetSafeStopResetType "956a2bdbbc2e1317a58568aa311b17b7")


