;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::MoveHome)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'MoveHome (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::MOVEHOME")
  (make-package "DSR_MSGS::MOVEHOME"))
(unless (find-package "DSR_MSGS::MOVEHOMEREQUEST")
  (make-package "DSR_MSGS::MOVEHOMEREQUEST"))
(unless (find-package "DSR_MSGS::MOVEHOMERESPONSE")
  (make-package "DSR_MSGS::MOVEHOMERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::MoveHomeRequest
  :super ros::object
  :slots (_target ))

(defmethod dsr_msgs::MoveHomeRequest
  (:init
   (&key
    ((:target __target) 0)
    )
   (send-super :init)
   (setq _target (round __target))
   self)
  (:target
   (&optional __target)
   (if __target (setq _target __target)) _target)
  (:serialization-length
   ()
   (+
    ;; int8 _target
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _target
       (write-byte _target s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _target
     (setq _target (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _target 127) (setq _target (- _target 256)))
   ;;
   self)
  )

(defclass dsr_msgs::MoveHomeResponse
  :super ros::object
  :slots (_res _success ))

(defmethod dsr_msgs::MoveHomeResponse
  (:init
   (&key
    ((:res __res) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _res (round __res))
   (setq _success __success)
   self)
  (:res
   (&optional __res)
   (if __res (setq _res __res)) _res)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int8 _res
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _res
       (write-byte _res s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _res
     (setq _res (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _res 127) (setq _res (- _res 256)))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::MoveHome
  :super ros::object
  :slots ())

(setf (get dsr_msgs::MoveHome :md5sum-) "1df045e08f5aa30b46232621f50af2fd")
(setf (get dsr_msgs::MoveHome :datatype-) "dsr_msgs/MoveHome")
(setf (get dsr_msgs::MoveHome :request) dsr_msgs::MoveHomeRequest)
(setf (get dsr_msgs::MoveHome :response) dsr_msgs::MoveHomeResponse)

(defmethod dsr_msgs::MoveHomeRequest
  (:response () (instance dsr_msgs::MoveHomeResponse :init)))

(setf (get dsr_msgs::MoveHomeRequest :md5sum-) "1df045e08f5aa30b46232621f50af2fd")
(setf (get dsr_msgs::MoveHomeRequest :datatype-) "dsr_msgs/MoveHomeRequest")
(setf (get dsr_msgs::MoveHomeRequest :definition-)
      "#____________________________________________________________________________________________
# move_home
# Homing is performed by moving to the joint motion to the mechanical or user defined home position.
# According to the input parameter [target], it moves to the mechanical home defined in the system or the home set by the user.
#____________________________________________________________________________________________

int8       target           # DR_HOME_TARGET_MECHANIC(0) : Mechanical home, joint angle (0,0,0,0,0,0)
                            # DR_HOME_TARGET_USER(1)     : user home
---
int8       res              # 0=success, otherwise fail 
bool       success

")

(setf (get dsr_msgs::MoveHomeResponse :md5sum-) "1df045e08f5aa30b46232621f50af2fd")
(setf (get dsr_msgs::MoveHomeResponse :datatype-) "dsr_msgs/MoveHomeResponse")
(setf (get dsr_msgs::MoveHomeResponse :definition-)
      "#____________________________________________________________________________________________
# move_home
# Homing is performed by moving to the joint motion to the mechanical or user defined home position.
# According to the input parameter [target], it moves to the mechanical home defined in the system or the home set by the user.
#____________________________________________________________________________________________

int8       target           # DR_HOME_TARGET_MECHANIC(0) : Mechanical home, joint angle (0,0,0,0,0,0)
                            # DR_HOME_TARGET_USER(1)     : user home
---
int8       res              # 0=success, otherwise fail 
bool       success

")



(provide :dsr_msgs/MoveHome "1df045e08f5aa30b46232621f50af2fd")


