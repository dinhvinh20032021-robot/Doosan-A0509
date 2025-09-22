;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::DrlResume)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'DrlResume (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::DRLRESUME")
  (make-package "DSR_MSGS::DRLRESUME"))
(unless (find-package "DSR_MSGS::DRLRESUMEREQUEST")
  (make-package "DSR_MSGS::DRLRESUMEREQUEST"))
(unless (find-package "DSR_MSGS::DRLRESUMERESPONSE")
  (make-package "DSR_MSGS::DRLRESUMERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::DrlResumeRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::DrlResumeRequest
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

(defclass dsr_msgs::DrlResumeResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::DrlResumeResponse
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

(defclass dsr_msgs::DrlResume
  :super ros::object
  :slots ())

(setf (get dsr_msgs::DrlResume :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::DrlResume :datatype-) "dsr_msgs/DrlResume")
(setf (get dsr_msgs::DrlResume :request) dsr_msgs::DrlResumeRequest)
(setf (get dsr_msgs::DrlResume :response) dsr_msgs::DrlResumeResponse)

(defmethod dsr_msgs::DrlResumeRequest
  (:response () (instance dsr_msgs::DrlResumeResponse :init)))

(setf (get dsr_msgs::DrlResumeRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::DrlResumeRequest :datatype-) "dsr_msgs/DrlResumeRequest")
(setf (get dsr_msgs::DrlResumeRequest :definition-)
      "#____________________________________________________________________________________________
# drl_script_resume  
# It is a service to resume the currently paused DRL program in the robot controller.    
#____________________________________________________________________________________________

---
bool success
")

(setf (get dsr_msgs::DrlResumeResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::DrlResumeResponse :datatype-) "dsr_msgs/DrlResumeResponse")
(setf (get dsr_msgs::DrlResumeResponse :definition-)
      "#____________________________________________________________________________________________
# drl_script_resume  
# It is a service to resume the currently paused DRL program in the robot controller.    
#____________________________________________________________________________________________

---
bool success
")



(provide :dsr_msgs/DrlResume "358e233cde0c8a8bcfea4ce193f8fc15")


