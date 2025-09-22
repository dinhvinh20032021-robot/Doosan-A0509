;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::MoveResume)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'MoveResume (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::MOVERESUME")
  (make-package "DSR_MSGS::MOVERESUME"))
(unless (find-package "DSR_MSGS::MOVERESUMEREQUEST")
  (make-package "DSR_MSGS::MOVERESUMEREQUEST"))
(unless (find-package "DSR_MSGS::MOVERESUMERESPONSE")
  (make-package "DSR_MSGS::MOVERESUMERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::MoveResumeRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::MoveResumeRequest
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

(defclass dsr_msgs::MoveResumeResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::MoveResumeResponse
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

(defclass dsr_msgs::MoveResume
  :super ros::object
  :slots ())

(setf (get dsr_msgs::MoveResume :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::MoveResume :datatype-) "dsr_msgs/MoveResume")
(setf (get dsr_msgs::MoveResume :request) dsr_msgs::MoveResumeRequest)
(setf (get dsr_msgs::MoveResume :response) dsr_msgs::MoveResumeResponse)

(defmethod dsr_msgs::MoveResumeRequest
  (:response () (instance dsr_msgs::MoveResumeResponse :init)))

(setf (get dsr_msgs::MoveResumeRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::MoveResumeRequest :datatype-) "dsr_msgs/MoveResumeRequest")
(setf (get dsr_msgs::MoveResumeRequest :definition-)
      "#____________________________________________________________________________________________
# motion pause
#____________________________________________________________________________________________

---
bool success

")

(setf (get dsr_msgs::MoveResumeResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::MoveResumeResponse :datatype-) "dsr_msgs/MoveResumeResponse")
(setf (get dsr_msgs::MoveResumeResponse :definition-)
      "#____________________________________________________________________________________________
# motion pause
#____________________________________________________________________________________________

---
bool success

")



(provide :dsr_msgs/MoveResume "358e233cde0c8a8bcfea4ce193f8fc15")


