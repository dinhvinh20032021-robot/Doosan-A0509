;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::MoveStop)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'MoveStop (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::MOVESTOP")
  (make-package "DSR_MSGS::MOVESTOP"))
(unless (find-package "DSR_MSGS::MOVESTOPREQUEST")
  (make-package "DSR_MSGS::MOVESTOPREQUEST"))
(unless (find-package "DSR_MSGS::MOVESTOPRESPONSE")
  (make-package "DSR_MSGS::MOVESTOPRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::MoveStopRequest
  :super ros::object
  :slots (_stop_mode ))

(defmethod dsr_msgs::MoveStopRequest
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

(defclass dsr_msgs::MoveStopResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::MoveStopResponse
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

(defclass dsr_msgs::MoveStop
  :super ros::object
  :slots ())

(setf (get dsr_msgs::MoveStop :md5sum-) "a0a25e9e76d4431108e6e2682aa29331")
(setf (get dsr_msgs::MoveStop :datatype-) "dsr_msgs/MoveStop")
(setf (get dsr_msgs::MoveStop :request) dsr_msgs::MoveStopRequest)
(setf (get dsr_msgs::MoveStop :response) dsr_msgs::MoveStopResponse)

(defmethod dsr_msgs::MoveStopRequest
  (:response () (instance dsr_msgs::MoveStopResponse :init)))

(setf (get dsr_msgs::MoveStopRequest :md5sum-) "a0a25e9e76d4431108e6e2682aa29331")
(setf (get dsr_msgs::MoveStopRequest :datatype-) "dsr_msgs/MoveStopRequest")
(setf (get dsr_msgs::MoveStopRequest :definition-)
      "#____________________________________________________________________________________________
# stop()
# 인자 설명 추가 필요!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#____________________________________________________________________________________________

int32 stop_mode         # DR_QSTOP_STO(0) : Quick stop (Stop Category 1 without STO(Safe Torque Off)
                        # DR_QSTOP(1)     : Quick stop (Stop Category 2)
                        # DR_SSTO(2)      : Soft Stop
                        # DR_HOLD(3)      : HOLD stop
---
bool success

")

(setf (get dsr_msgs::MoveStopResponse :md5sum-) "a0a25e9e76d4431108e6e2682aa29331")
(setf (get dsr_msgs::MoveStopResponse :datatype-) "dsr_msgs/MoveStopResponse")
(setf (get dsr_msgs::MoveStopResponse :definition-)
      "#____________________________________________________________________________________________
# stop()
# 인자 설명 추가 필요!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#____________________________________________________________________________________________

int32 stop_mode         # DR_QSTOP_STO(0) : Quick stop (Stop Category 1 without STO(Safe Torque Off)
                        # DR_QSTOP(1)     : Quick stop (Stop Category 2)
                        # DR_SSTO(2)      : Soft Stop
                        # DR_HOLD(3)      : HOLD stop
---
bool success

")



(provide :dsr_msgs/MoveStop "a0a25e9e76d4431108e6e2682aa29331")


