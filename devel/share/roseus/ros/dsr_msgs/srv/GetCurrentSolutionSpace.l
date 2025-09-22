;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetCurrentSolutionSpace)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetCurrentSolutionSpace (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETCURRENTSOLUTIONSPACE")
  (make-package "DSR_MSGS::GETCURRENTSOLUTIONSPACE"))
(unless (find-package "DSR_MSGS::GETCURRENTSOLUTIONSPACEREQUEST")
  (make-package "DSR_MSGS::GETCURRENTSOLUTIONSPACEREQUEST"))
(unless (find-package "DSR_MSGS::GETCURRENTSOLUTIONSPACERESPONSE")
  (make-package "DSR_MSGS::GETCURRENTSOLUTIONSPACERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetCurrentSolutionSpaceRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetCurrentSolutionSpaceRequest
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

(defclass dsr_msgs::GetCurrentSolutionSpaceResponse
  :super ros::object
  :slots (_sol_space _success ))

(defmethod dsr_msgs::GetCurrentSolutionSpaceResponse
  (:init
   (&key
    ((:sol_space __sol_space) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _sol_space (round __sol_space))
   (setq _success __success)
   self)
  (:sol_space
   (&optional __sol_space)
   (if __sol_space (setq _sol_space __sol_space)) _sol_space)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int8 _sol_space
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _sol_space
       (write-byte _sol_space s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _sol_space
     (setq _sol_space (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _sol_space 127) (setq _sol_space (- _sol_space 256)))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetCurrentSolutionSpace
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetCurrentSolutionSpace :md5sum-) "bacee50609041091b8cb888ffb4bd562")
(setf (get dsr_msgs::GetCurrentSolutionSpace :datatype-) "dsr_msgs/GetCurrentSolutionSpace")
(setf (get dsr_msgs::GetCurrentSolutionSpace :request) dsr_msgs::GetCurrentSolutionSpaceRequest)
(setf (get dsr_msgs::GetCurrentSolutionSpace :response) dsr_msgs::GetCurrentSolutionSpaceResponse)

(defmethod dsr_msgs::GetCurrentSolutionSpaceRequest
  (:response () (instance dsr_msgs::GetCurrentSolutionSpaceResponse :init)))

(setf (get dsr_msgs::GetCurrentSolutionSpaceRequest :md5sum-) "bacee50609041091b8cb888ffb4bd562")
(setf (get dsr_msgs::GetCurrentSolutionSpaceRequest :datatype-) "dsr_msgs/GetCurrentSolutionSpaceRequest")
(setf (get dsr_msgs::GetCurrentSolutionSpaceRequest :definition-)
      "#____________________________________________________________________________________________
# get_current_solution_space
#____________________________________________________________________________________________
# This service returns the current solution space value.

---
int8        sol_space         # solution space : 0 ~ 7
bool        success
")

(setf (get dsr_msgs::GetCurrentSolutionSpaceResponse :md5sum-) "bacee50609041091b8cb888ffb4bd562")
(setf (get dsr_msgs::GetCurrentSolutionSpaceResponse :datatype-) "dsr_msgs/GetCurrentSolutionSpaceResponse")
(setf (get dsr_msgs::GetCurrentSolutionSpaceResponse :definition-)
      "#____________________________________________________________________________________________
# get_current_solution_space
#____________________________________________________________________________________________
# This service returns the current solution space value.

---
int8        sol_space         # solution space : 0 ~ 7
bool        success
")



(provide :dsr_msgs/GetCurrentSolutionSpace "bacee50609041091b8cb888ffb4bd562")


