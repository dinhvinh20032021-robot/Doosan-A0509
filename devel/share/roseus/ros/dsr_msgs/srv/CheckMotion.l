;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::CheckMotion)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'CheckMotion (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::CHECKMOTION")
  (make-package "DSR_MSGS::CHECKMOTION"))
(unless (find-package "DSR_MSGS::CHECKMOTIONREQUEST")
  (make-package "DSR_MSGS::CHECKMOTIONREQUEST"))
(unless (find-package "DSR_MSGS::CHECKMOTIONRESPONSE")
  (make-package "DSR_MSGS::CHECKMOTIONRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::CheckMotionRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::CheckMotionRequest
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

(defclass dsr_msgs::CheckMotionResponse
  :super ros::object
  :slots (_status _success ))

(defmethod dsr_msgs::CheckMotionResponse
  (:init
   (&key
    ((:status __status) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _status (round __status))
   (setq _success __success)
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int8 _status
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _status
       (write-byte _status s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _status 127) (setq _status (- _status 256)))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::CheckMotion
  :super ros::object
  :slots ())

(setf (get dsr_msgs::CheckMotion :md5sum-) "f2991389d99138c3293e56c6922ac005")
(setf (get dsr_msgs::CheckMotion :datatype-) "dsr_msgs/CheckMotion")
(setf (get dsr_msgs::CheckMotion :request) dsr_msgs::CheckMotionRequest)
(setf (get dsr_msgs::CheckMotion :response) dsr_msgs::CheckMotionResponse)

(defmethod dsr_msgs::CheckMotionRequest
  (:response () (instance dsr_msgs::CheckMotionResponse :init)))

(setf (get dsr_msgs::CheckMotionRequest :md5sum-) "f2991389d99138c3293e56c6922ac005")
(setf (get dsr_msgs::CheckMotionRequest :datatype-) "dsr_msgs/CheckMotionRequest")
(setf (get dsr_msgs::CheckMotionRequest :definition-)
      "#____________________________________________________________________________________________
# check_motion
# return status of the currently active motion.
# Homing is performed by moving to the joint motion to the mechanical or user defined home position.
# According to the input parameter [target], it moves to the mechanical home defined in the system or the home set by the user.
#____________________________________________________________________________________________

---
int8       status          # DR_STATE_IDLE(0) : no motion in action
                           # DR_STATE_INIT(1) : motion being calculated
                           # DR_STATE_BUSY(2) : motion in operation
bool       success 
")

(setf (get dsr_msgs::CheckMotionResponse :md5sum-) "f2991389d99138c3293e56c6922ac005")
(setf (get dsr_msgs::CheckMotionResponse :datatype-) "dsr_msgs/CheckMotionResponse")
(setf (get dsr_msgs::CheckMotionResponse :definition-)
      "#____________________________________________________________________________________________
# check_motion
# return status of the currently active motion.
# Homing is performed by moving to the joint motion to the mechanical or user defined home position.
# According to the input parameter [target], it moves to the mechanical home defined in the system or the home set by the user.
#____________________________________________________________________________________________

---
int8       status          # DR_STATE_IDLE(0) : no motion in action
                           # DR_STATE_INIT(1) : motion being calculated
                           # DR_STATE_BUSY(2) : motion in operation
bool       success 
")



(provide :dsr_msgs/CheckMotion "f2991389d99138c3293e56c6922ac005")


