;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::MovePause)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'MovePause (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::MOVEPAUSE")
  (make-package "DSR_MSGS::MOVEPAUSE"))
(unless (find-package "DSR_MSGS::MOVEPAUSEREQUEST")
  (make-package "DSR_MSGS::MOVEPAUSEREQUEST"))
(unless (find-package "DSR_MSGS::MOVEPAUSERESPONSE")
  (make-package "DSR_MSGS::MOVEPAUSERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::MovePauseRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::MovePauseRequest
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

(defclass dsr_msgs::MovePauseResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::MovePauseResponse
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

(defclass dsr_msgs::MovePause
  :super ros::object
  :slots ())

(setf (get dsr_msgs::MovePause :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::MovePause :datatype-) "dsr_msgs/MovePause")
(setf (get dsr_msgs::MovePause :request) dsr_msgs::MovePauseRequest)
(setf (get dsr_msgs::MovePause :response) dsr_msgs::MovePauseResponse)

(defmethod dsr_msgs::MovePauseRequest
  (:response () (instance dsr_msgs::MovePauseResponse :init)))

(setf (get dsr_msgs::MovePauseRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::MovePauseRequest :datatype-) "dsr_msgs/MovePauseRequest")
(setf (get dsr_msgs::MovePauseRequest :definition-)
      "#____________________________________________________________________________________________
# motion pause
#____________________________________________________________________________________________

---
bool success

")

(setf (get dsr_msgs::MovePauseResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::MovePauseResponse :datatype-) "dsr_msgs/MovePauseResponse")
(setf (get dsr_msgs::MovePauseResponse :definition-)
      "#____________________________________________________________________________________________
# motion pause
#____________________________________________________________________________________________

---
bool success

")



(provide :dsr_msgs/MovePause "358e233cde0c8a8bcfea4ce193f8fc15")


