;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::RobotStop)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'RobotStop (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::ROBOTSTOP")
  (make-package "DSR_MSGS::ROBOTSTOP"))

(in-package "ROS")
;;//! \htmlinclude RobotStop.msg.html


(defclass dsr_msgs::RobotStop
  :super ros::object
  :slots (_stop_mode ))

(defmethod dsr_msgs::RobotStop
  (:init
   (&key
    ((:stop_mode __stop_mode) 0)
    )
   (send-super :init)
   (setq _stop_mode (round __stop_mode))
   self)
  (:stop_mode
   (&optional __stop_mode)
   (if __stop_mode (setq _stop_mode __stop_mode)) _stop_mode)
  (:serialization-length
   ()
   (+
    ;; int32 _stop_mode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _stop_mode
       (write-long _stop_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _stop_mode
     (setq _stop_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dsr_msgs::RobotStop :md5sum-) "82712390efeed0d0668a551e004b332c")
(setf (get dsr_msgs::RobotStop :datatype-) "dsr_msgs/RobotStop")
(setf (get dsr_msgs::RobotStop :definition-)
      "#____________________________________________________________________________________________
# [ robot stop mode ] 
# 0 : STOP_TYPE_QUICK_STO
# 1 : STOP_TYPE_QUICK
# 2 : STOP_TYPE_SLOW
# 3 : STOP_TYPE_HOLD = STOP_TYPE_EMERGENCY
#
#____________________________________________________________________________________________

int32 stop_mode  

")



(provide :dsr_msgs/RobotStop "82712390efeed0d0668a551e004b332c")


