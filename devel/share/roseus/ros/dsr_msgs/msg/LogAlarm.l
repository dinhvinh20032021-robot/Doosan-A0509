;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::LogAlarm)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'LogAlarm (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::LOGALARM")
  (make-package "DSR_MSGS::LOGALARM"))

(in-package "ROS")
;;//! \htmlinclude LogAlarm.msg.html


(defclass dsr_msgs::LogAlarm
  :super ros::object
  :slots (_level _group _index _param ))

(defmethod dsr_msgs::LogAlarm
  (:init
   (&key
    ((:level __level) 0)
    ((:group __group) 0)
    ((:index __index) 0)
    ((:param __param) (let (r) (dotimes (i 3) (push "" r)) r))
    )
   (send-super :init)
   (setq _level (round __level))
   (setq _group (round __group))
   (setq _index (round __index))
   (setq _param __param)
   self)
  (:level
   (&optional __level)
   (if __level (setq _level __level)) _level)
  (:group
   (&optional __group)
   (if __group (setq _group __group)) _group)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:param
   (&optional __param)
   (if __param (setq _param __param)) _param)
  (:serialization-length
   ()
   (+
    ;; int32 _level
    4
    ;; int32 _group
    4
    ;; int32 _index
    4
    ;; string[3] _param
    (apply #'+ 4 (length _param)    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _level
       (write-long _level s)
     ;; int32 _group
       (write-long _group s)
     ;; int32 _index
       (write-long _index s)
     ;; string[3] _param
     (write-long (length _param) s)
     (dolist (elem _param)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _level
     (setq _level (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _group
     (setq _group (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _index
     (setq _index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string[3] _param
   (dotimes (i (length _param))
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _param i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     )
   ;;
   self)
  )

(setf (get dsr_msgs::LogAlarm :md5sum-) "c127c7c1149264259595eb8c3ff9972c")
(setf (get dsr_msgs::LogAlarm :datatype-) "dsr_msgs/LogAlarm")
(setf (get dsr_msgs::LogAlarm :definition-)
      "#____________________________________________________________________________________________
# log of alarm
#____________________________________________________________________________________________

int32         level
int32         group
int32         index
string[3]     param
")



(provide :dsr_msgs/LogAlarm "c127c7c1149264259595eb8c3ff9972c")


