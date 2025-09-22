;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetDrlState)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetDrlState (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETDRLSTATE")
  (make-package "DSR_MSGS::GETDRLSTATE"))
(unless (find-package "DSR_MSGS::GETDRLSTATEREQUEST")
  (make-package "DSR_MSGS::GETDRLSTATEREQUEST"))
(unless (find-package "DSR_MSGS::GETDRLSTATERESPONSE")
  (make-package "DSR_MSGS::GETDRLSTATERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetDrlStateRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetDrlStateRequest
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

(defclass dsr_msgs::GetDrlStateResponse
  :super ros::object
  :slots (_drl_state _success ))

(defmethod dsr_msgs::GetDrlStateResponse
  (:init
   (&key
    ((:drl_state __drl_state) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _drl_state (round __drl_state))
   (setq _success __success)
   self)
  (:drl_state
   (&optional __drl_state)
   (if __drl_state (setq _drl_state __drl_state)) _drl_state)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int8 _drl_state
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _drl_state
       (write-byte _drl_state s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _drl_state
     (setq _drl_state (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _drl_state 127) (setq _drl_state (- _drl_state 256)))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetDrlState
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetDrlState :md5sum-) "9dd00c5d3b26eace3288075b31af3114")
(setf (get dsr_msgs::GetDrlState :datatype-) "dsr_msgs/GetDrlState")
(setf (get dsr_msgs::GetDrlState :request) dsr_msgs::GetDrlStateRequest)
(setf (get dsr_msgs::GetDrlState :response) dsr_msgs::GetDrlStateResponse)

(defmethod dsr_msgs::GetDrlStateRequest
  (:response () (instance dsr_msgs::GetDrlStateResponse :init)))

(setf (get dsr_msgs::GetDrlStateRequest :md5sum-) "9dd00c5d3b26eace3288075b31af3114")
(setf (get dsr_msgs::GetDrlStateRequest :datatype-) "dsr_msgs/GetDrlStateRequest")
(setf (get dsr_msgs::GetDrlStateRequest :definition-)
      "#____________________________________________________________________________________________
# get_drl_state
# Get DRL Program State
# 0 : DRL_PROGRAM_STATE_PLAY
# 1 : DRL_PROGRAM_STATE_STOP
# 2 : DRL_PROGRAM_STATE_HOLD
# 3 : DRL_PROGRAM_STATE_LAST
# drfl.GetProgramState()
#____________________________________________________________________________________________

---
int8        drl_state # <DRL_PROGRAM_STATE>
bool        success
")

(setf (get dsr_msgs::GetDrlStateResponse :md5sum-) "9dd00c5d3b26eace3288075b31af3114")
(setf (get dsr_msgs::GetDrlStateResponse :datatype-) "dsr_msgs/GetDrlStateResponse")
(setf (get dsr_msgs::GetDrlStateResponse :definition-)
      "#____________________________________________________________________________________________
# get_drl_state
# Get DRL Program State
# 0 : DRL_PROGRAM_STATE_PLAY
# 1 : DRL_PROGRAM_STATE_STOP
# 2 : DRL_PROGRAM_STATE_HOLD
# 3 : DRL_PROGRAM_STATE_LAST
# drfl.GetProgramState()
#____________________________________________________________________________________________

---
int8        drl_state # <DRL_PROGRAM_STATE>
bool        success
")



(provide :dsr_msgs/GetDrlState "9dd00c5d3b26eace3288075b31af3114")


