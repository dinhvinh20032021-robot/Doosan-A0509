;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ReleaseForce)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ReleaseForce (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::RELEASEFORCE")
  (make-package "DSR_MSGS::RELEASEFORCE"))
(unless (find-package "DSR_MSGS::RELEASEFORCEREQUEST")
  (make-package "DSR_MSGS::RELEASEFORCEREQUEST"))
(unless (find-package "DSR_MSGS::RELEASEFORCERESPONSE")
  (make-package "DSR_MSGS::RELEASEFORCERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::ReleaseForceRequest
  :super ros::object
  :slots (_time ))

(defmethod dsr_msgs::ReleaseForceRequest
  (:init
   (&key
    ((:time __time) 0.0)
    )
   (send-super :init)
   (setq _time (float __time))
   self)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; float64 _time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass dsr_msgs::ReleaseForceResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::ReleaseForceResponse
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

(defclass dsr_msgs::ReleaseForce
  :super ros::object
  :slots ())

(setf (get dsr_msgs::ReleaseForce :md5sum-) "54a0fd05cf7c093a898987563c9ea327")
(setf (get dsr_msgs::ReleaseForce :datatype-) "dsr_msgs/ReleaseForce")
(setf (get dsr_msgs::ReleaseForce :request) dsr_msgs::ReleaseForceRequest)
(setf (get dsr_msgs::ReleaseForce :response) dsr_msgs::ReleaseForceResponse)

(defmethod dsr_msgs::ReleaseForceRequest
  (:response () (instance dsr_msgs::ReleaseForceResponse :init)))

(setf (get dsr_msgs::ReleaseForceRequest :md5sum-) "54a0fd05cf7c093a898987563c9ea327")
(setf (get dsr_msgs::ReleaseForceRequest :datatype-) "dsr_msgs/ReleaseForceRequest")
(setf (get dsr_msgs::ReleaseForceRequest :definition-)
      "#____________________________________________________________________________________________
# release_force  
#____________________________________________________________________________________________

float64    time # 0          # Time needed to reduce the force (0 ~ 1.0) 
---
bool       success 
")

(setf (get dsr_msgs::ReleaseForceResponse :md5sum-) "54a0fd05cf7c093a898987563c9ea327")
(setf (get dsr_msgs::ReleaseForceResponse :datatype-) "dsr_msgs/ReleaseForceResponse")
(setf (get dsr_msgs::ReleaseForceResponse :definition-)
      "#____________________________________________________________________________________________
# release_force  
#____________________________________________________________________________________________

float64    time # 0          # Time needed to reduce the force (0 ~ 1.0) 
---
bool       success 
")



(provide :dsr_msgs/ReleaseForce "54a0fd05cf7c093a898987563c9ea327")


