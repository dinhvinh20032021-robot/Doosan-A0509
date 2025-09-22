;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::RobotError)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'RobotError (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::ROBOTERROR")
  (make-package "DSR_MSGS::ROBOTERROR"))

(in-package "ROS")
;;//! \htmlinclude RobotError.msg.html


(defclass dsr_msgs::RobotError
  :super ros::object
  :slots (_level _group _code _msg1 _msg2 _msg3 ))

(defmethod dsr_msgs::RobotError
  (:init
   (&key
    ((:level __level) 0)
    ((:group __group) 0)
    ((:code __code) 0)
    ((:msg1 __msg1) "")
    ((:msg2 __msg2) "")
    ((:msg3 __msg3) "")
    )
   (send-super :init)
   (setq _level (round __level))
   (setq _group (round __group))
   (setq _code (round __code))
   (setq _msg1 (string __msg1))
   (setq _msg2 (string __msg2))
   (setq _msg3 (string __msg3))
   self)
  (:level
   (&optional __level)
   (if __level (setq _level __level)) _level)
  (:group
   (&optional __group)
   (if __group (setq _group __group)) _group)
  (:code
   (&optional __code)
   (if __code (setq _code __code)) _code)
  (:msg1
   (&optional __msg1)
   (if __msg1 (setq _msg1 __msg1)) _msg1)
  (:msg2
   (&optional __msg2)
   (if __msg2 (setq _msg2 __msg2)) _msg2)
  (:msg3
   (&optional __msg3)
   (if __msg3 (setq _msg3 __msg3)) _msg3)
  (:serialization-length
   ()
   (+
    ;; int32 _level
    4
    ;; int32 _group
    4
    ;; int32 _code
    4
    ;; string _msg1
    4 (length _msg1)
    ;; string _msg2
    4 (length _msg2)
    ;; string _msg3
    4 (length _msg3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _level
       (write-long _level s)
     ;; int32 _group
       (write-long _group s)
     ;; int32 _code
       (write-long _code s)
     ;; string _msg1
       (write-long (length _msg1) s) (princ _msg1 s)
     ;; string _msg2
       (write-long (length _msg2) s) (princ _msg2 s)
     ;; string _msg3
       (write-long (length _msg3) s) (princ _msg3 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _level
     (setq _level (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _group
     (setq _group (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _code
     (setq _code (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _msg1
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msg1 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _msg2
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msg2 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _msg3
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msg3 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get dsr_msgs::RobotError :md5sum-) "2d9680ace6867f719d7e19bbc321e6da")
(setf (get dsr_msgs::RobotError :datatype-) "dsr_msgs/RobotError")
(setf (get dsr_msgs::RobotError :definition-)
      "#____________________________________________________________________________________________
# [ robot error msg ] 
#____________________________________________________________________________________________

int32	    level   # INFO =1, WARN =2, ERROR =3 
int32	    group   # SYSTEM =1, MOTION =2, TP =3, INVERTER =4, SAFETY_CONTROLLER =5   
int32	    code    # error code 
string      msg1    # error msg 1
string      msg2    # error msg 2
string      msg3    # error msg 3
")



(provide :dsr_msgs/RobotError "2d9680ace6867f719d7e19bbc321e6da")


