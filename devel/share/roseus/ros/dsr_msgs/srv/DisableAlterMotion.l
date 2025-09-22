;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::DisableAlterMotion)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'DisableAlterMotion (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::DISABLEALTERMOTION")
  (make-package "DSR_MSGS::DISABLEALTERMOTION"))
(unless (find-package "DSR_MSGS::DISABLEALTERMOTIONREQUEST")
  (make-package "DSR_MSGS::DISABLEALTERMOTIONREQUEST"))
(unless (find-package "DSR_MSGS::DISABLEALTERMOTIONRESPONSE")
  (make-package "DSR_MSGS::DISABLEALTERMOTIONRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::DisableAlterMotionRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::DisableAlterMotionRequest
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

(defclass dsr_msgs::DisableAlterMotionResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::DisableAlterMotionResponse
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

(defclass dsr_msgs::DisableAlterMotion
  :super ros::object
  :slots ())

(setf (get dsr_msgs::DisableAlterMotion :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::DisableAlterMotion :datatype-) "dsr_msgs/DisableAlterMotion")
(setf (get dsr_msgs::DisableAlterMotion :request) dsr_msgs::DisableAlterMotionRequest)
(setf (get dsr_msgs::DisableAlterMotion :response) dsr_msgs::DisableAlterMotionResponse)

(defmethod dsr_msgs::DisableAlterMotionRequest
  (:response () (instance dsr_msgs::DisableAlterMotionResponse :init)))

(setf (get dsr_msgs::DisableAlterMotionRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::DisableAlterMotionRequest :datatype-) "dsr_msgs/DisableAlterMotionRequest")
(setf (get dsr_msgs::DisableAlterMotionRequest :definition-)
      "#____________________________________________________________________________________________
# disable_alter_motion  
# deactivates alter motion
#____________________________________________________________________________________________

---
bool success
")

(setf (get dsr_msgs::DisableAlterMotionResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::DisableAlterMotionResponse :datatype-) "dsr_msgs/DisableAlterMotionResponse")
(setf (get dsr_msgs::DisableAlterMotionResponse :definition-)
      "#____________________________________________________________________________________________
# disable_alter_motion  
# deactivates alter motion
#____________________________________________________________________________________________

---
bool success
")



(provide :dsr_msgs/DisableAlterMotion "358e233cde0c8a8bcfea4ce193f8fc15")


