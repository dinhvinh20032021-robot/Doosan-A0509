;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetLastAlarm)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetLastAlarm (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETLASTALARM")
  (make-package "DSR_MSGS::GETLASTALARM"))
(unless (find-package "DSR_MSGS::GETLASTALARMREQUEST")
  (make-package "DSR_MSGS::GETLASTALARMREQUEST"))
(unless (find-package "DSR_MSGS::GETLASTALARMRESPONSE")
  (make-package "DSR_MSGS::GETLASTALARMRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetLastAlarmRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetLastAlarmRequest
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

(defclass dsr_msgs::GetLastAlarmResponse
  :super ros::object
  :slots (_log_alarm _success ))

(defmethod dsr_msgs::GetLastAlarmResponse
  (:init
   (&key
    ((:log_alarm __log_alarm) (instance dsr_msgs::LogAlarm :init))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _log_alarm __log_alarm)
   (setq _success __success)
   self)
  (:log_alarm
   (&rest __log_alarm)
   (if (keywordp (car __log_alarm))
       (send* _log_alarm __log_alarm)
     (progn
       (if __log_alarm (setq _log_alarm (car __log_alarm)))
       _log_alarm)))
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; dsr_msgs/LogAlarm _log_alarm
    (send _log_alarm :serialization-length)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; dsr_msgs/LogAlarm _log_alarm
       (send _log_alarm :serialize s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; dsr_msgs/LogAlarm _log_alarm
     (send _log_alarm :deserialize buf ptr-) (incf ptr- (send _log_alarm :serialization-length))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetLastAlarm
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetLastAlarm :md5sum-) "a6d300dc8ba91bb8020a696fecebd6f2")
(setf (get dsr_msgs::GetLastAlarm :datatype-) "dsr_msgs/GetLastAlarm")
(setf (get dsr_msgs::GetLastAlarm :request) dsr_msgs::GetLastAlarmRequest)
(setf (get dsr_msgs::GetLastAlarm :response) dsr_msgs::GetLastAlarmResponse)

(defmethod dsr_msgs::GetLastAlarmRequest
  (:response () (instance dsr_msgs::GetLastAlarmResponse :init)))

(setf (get dsr_msgs::GetLastAlarmRequest :md5sum-) "a6d300dc8ba91bb8020a696fecebd6f2")
(setf (get dsr_msgs::GetLastAlarmRequest :datatype-) "dsr_msgs/GetLastAlarmRequest")
(setf (get dsr_msgs::GetLastAlarmRequest :definition-)
      "#____________________________________________________________________________________________
# get_last_alarm
###Struct.LOG_ARARM
#____________________________________________________________________________________________

---
LogAlarm log_alarm
bool        success

================================================================================
MSG: dsr_msgs/LogAlarm
#____________________________________________________________________________________________
# log of alarm
#____________________________________________________________________________________________

int32         level
int32         group
int32         index
")

(setf (get dsr_msgs::GetLastAlarmResponse :md5sum-) "a6d300dc8ba91bb8020a696fecebd6f2")
(setf (get dsr_msgs::GetLastAlarmResponse :datatype-) "dsr_msgs/GetLastAlarmResponse")
(setf (get dsr_msgs::GetLastAlarmResponse :definition-)
      "#____________________________________________________________________________________________
# get_last_alarm
###Struct.LOG_ARARM
#____________________________________________________________________________________________

---
LogAlarm log_alarm
bool        success

================================================================================
MSG: dsr_msgs/LogAlarm
#____________________________________________________________________________________________
# log of alarm
#____________________________________________________________________________________________

int32         level
int32         group
int32         index
")



(provide :dsr_msgs/GetLastAlarm "a6d300dc8ba91bb8020a696fecebd6f2")


