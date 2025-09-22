;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetSingularityHandling)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetSingularityHandling (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETSINGULARITYHANDLING")
  (make-package "DSR_MSGS::SETSINGULARITYHANDLING"))
(unless (find-package "DSR_MSGS::SETSINGULARITYHANDLINGREQUEST")
  (make-package "DSR_MSGS::SETSINGULARITYHANDLINGREQUEST"))
(unless (find-package "DSR_MSGS::SETSINGULARITYHANDLINGRESPONSE")
  (make-package "DSR_MSGS::SETSINGULARITYHANDLINGRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetSingularityHandlingRequest
  :super ros::object
  :slots (_mode ))

(defmethod dsr_msgs::SetSingularityHandlingRequest
  (:init
   (&key
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; int8 _mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _mode
       (write-byte _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;;
   self)
  )

(defclass dsr_msgs::SetSingularityHandlingResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetSingularityHandlingResponse
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

(defclass dsr_msgs::SetSingularityHandling
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetSingularityHandling :md5sum-) "5da578ca824a6682cf704b593bb1a59e")
(setf (get dsr_msgs::SetSingularityHandling :datatype-) "dsr_msgs/SetSingularityHandling")
(setf (get dsr_msgs::SetSingularityHandling :request) dsr_msgs::SetSingularityHandlingRequest)
(setf (get dsr_msgs::SetSingularityHandling :response) dsr_msgs::SetSingularityHandlingResponse)

(defmethod dsr_msgs::SetSingularityHandlingRequest
  (:response () (instance dsr_msgs::SetSingularityHandlingResponse :init)))

(setf (get dsr_msgs::SetSingularityHandlingRequest :md5sum-) "5da578ca824a6682cf704b593bb1a59e")
(setf (get dsr_msgs::SetSingularityHandlingRequest :datatype-) "dsr_msgs/SetSingularityHandlingRequest")
(setf (get dsr_msgs::SetSingularityHandlingRequest :definition-)
      "#____________________________________________________________________________________________
# set_singularity_handling
# 
#____________________________________________________________________________________________

int8       mode         # DR_AVOID(0)     : Automatic avoidance mode
                        # DR_TASK_STOP(1) : Deceleration/ Warning/ Task termination
                        # DR_VAR_VEL(2)   : Variable velocity mode

---
bool success

")

(setf (get dsr_msgs::SetSingularityHandlingResponse :md5sum-) "5da578ca824a6682cf704b593bb1a59e")
(setf (get dsr_msgs::SetSingularityHandlingResponse :datatype-) "dsr_msgs/SetSingularityHandlingResponse")
(setf (get dsr_msgs::SetSingularityHandlingResponse :definition-)
      "#____________________________________________________________________________________________
# set_singularity_handling
# 
#____________________________________________________________________________________________

int8       mode         # DR_AVOID(0)     : Automatic avoidance mode
                        # DR_TASK_STOP(1) : Deceleration/ Warning/ Task termination
                        # DR_VAR_VEL(2)   : Variable velocity mode

---
bool success

")



(provide :dsr_msgs/SetSingularityHandling "5da578ca824a6682cf704b593bb1a59e")


