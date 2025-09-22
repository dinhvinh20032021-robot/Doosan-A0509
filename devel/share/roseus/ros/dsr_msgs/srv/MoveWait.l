;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::MoveWait)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'MoveWait (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::MOVEWAIT")
  (make-package "DSR_MSGS::MOVEWAIT"))
(unless (find-package "DSR_MSGS::MOVEWAITREQUEST")
  (make-package "DSR_MSGS::MOVEWAITREQUEST"))
(unless (find-package "DSR_MSGS::MOVEWAITRESPONSE")
  (make-package "DSR_MSGS::MOVEWAITRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::MoveWaitRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::MoveWaitRequest
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

(defclass dsr_msgs::MoveWaitResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::MoveWaitResponse
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

(defclass dsr_msgs::MoveWait
  :super ros::object
  :slots ())

(setf (get dsr_msgs::MoveWait :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::MoveWait :datatype-) "dsr_msgs/MoveWait")
(setf (get dsr_msgs::MoveWait :request) dsr_msgs::MoveWaitRequest)
(setf (get dsr_msgs::MoveWait :response) dsr_msgs::MoveWaitResponse)

(defmethod dsr_msgs::MoveWaitRequest
  (:response () (instance dsr_msgs::MoveWaitResponse :init)))

(setf (get dsr_msgs::MoveWaitRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::MoveWaitRequest :datatype-) "dsr_msgs/MoveWaitRequest")
(setf (get dsr_msgs::MoveWaitRequest :definition-)
      "#____________________________________________________________________________________________
# move_wait
# This Service sets the waiting time between the previous motion command 
# and the motion command in the next line.
#____________________________________________________________________________________________

---
bool success
")

(setf (get dsr_msgs::MoveWaitResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::MoveWaitResponse :datatype-) "dsr_msgs/MoveWaitResponse")
(setf (get dsr_msgs::MoveWaitResponse :definition-)
      "#____________________________________________________________________________________________
# move_wait
# This Service sets the waiting time between the previous motion command 
# and the motion command in the next line.
#____________________________________________________________________________________________

---
bool success
")



(provide :dsr_msgs/MoveWait "358e233cde0c8a8bcfea4ce193f8fc15")


