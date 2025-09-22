;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::DrlStop)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'DrlStop (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::DRLSTOP")
  (make-package "DSR_MSGS::DRLSTOP"))
(unless (find-package "DSR_MSGS::DRLSTOPREQUEST")
  (make-package "DSR_MSGS::DRLSTOPREQUEST"))
(unless (find-package "DSR_MSGS::DRLSTOPRESPONSE")
  (make-package "DSR_MSGS::DRLSTOPRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::DrlStopRequest
  :super ros::object
  :slots (_stop_mode ))

(defmethod dsr_msgs::DrlStopRequest
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
    ;; int8 _stop_mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _stop_mode
       (write-byte _stop_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _stop_mode
     (setq _stop_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _stop_mode 127) (setq _stop_mode (- _stop_mode 256)))
   ;;
   self)
  )

(defclass dsr_msgs::DrlStopResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::DrlStopResponse
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

(defclass dsr_msgs::DrlStop
  :super ros::object
  :slots ())

(setf (get dsr_msgs::DrlStop :md5sum-) "a7f8fadac11dc7732e1bdb5e5d128820")
(setf (get dsr_msgs::DrlStop :datatype-) "dsr_msgs/DrlStop")
(setf (get dsr_msgs::DrlStop :request) dsr_msgs::DrlStopRequest)
(setf (get dsr_msgs::DrlStop :response) dsr_msgs::DrlStopResponse)

(defmethod dsr_msgs::DrlStopRequest
  (:response () (instance dsr_msgs::DrlStopResponse :init)))

(setf (get dsr_msgs::DrlStopRequest :md5sum-) "a7f8fadac11dc7732e1bdb5e5d128820")
(setf (get dsr_msgs::DrlStopRequest :datatype-) "dsr_msgs/DrlStopRequest")
(setf (get dsr_msgs::DrlStopRequest :definition-)
      "#____________________________________________________________________________________________
# drl_script_stop  
# STOP_TYPE_QUICK_STO = 0
# STOP_TYPE_QUICK     = 1
# STOP_TYPE_SLOW      = 2
# STOP_TYPE_HOLD = STOP_TYPE_EMERGENCY = 3  
#____________________________________________________________________________________________

int8    stop_mode       # <STOP_TYPE> stop_mode       
---
bool    success
")

(setf (get dsr_msgs::DrlStopResponse :md5sum-) "a7f8fadac11dc7732e1bdb5e5d128820")
(setf (get dsr_msgs::DrlStopResponse :datatype-) "dsr_msgs/DrlStopResponse")
(setf (get dsr_msgs::DrlStopResponse :definition-)
      "#____________________________________________________________________________________________
# drl_script_stop  
# STOP_TYPE_QUICK_STO = 0
# STOP_TYPE_QUICK     = 1
# STOP_TYPE_SLOW      = 2
# STOP_TYPE_HOLD = STOP_TYPE_EMERGENCY = 3  
#____________________________________________________________________________________________

int8    stop_mode       # <STOP_TYPE> stop_mode       
---
bool    success
")



(provide :dsr_msgs/DrlStop "a7f8fadac11dc7732e1bdb5e5d128820")


