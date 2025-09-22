;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetControlSpace)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetControlSpace (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETCONTROLSPACE")
  (make-package "DSR_MSGS::GETCONTROLSPACE"))
(unless (find-package "DSR_MSGS::GETCONTROLSPACEREQUEST")
  (make-package "DSR_MSGS::GETCONTROLSPACEREQUEST"))
(unless (find-package "DSR_MSGS::GETCONTROLSPACERESPONSE")
  (make-package "DSR_MSGS::GETCONTROLSPACERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetControlSpaceRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetControlSpaceRequest
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

(defclass dsr_msgs::GetControlSpaceResponse
  :super ros::object
  :slots (_space _success ))

(defmethod dsr_msgs::GetControlSpaceResponse
  (:init
   (&key
    ((:space __space) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _space (round __space))
   (setq _success __success)
   self)
  (:space
   (&optional __space)
   (if __space (setq _space __space)) _space)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int8 _space
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _space
       (write-byte _space s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _space
     (setq _space (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _space 127) (setq _space (- _space 256)))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetControlSpace
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetControlSpace :md5sum-) "1a5780af2202e44900c27ba310bfe10c")
(setf (get dsr_msgs::GetControlSpace :datatype-) "dsr_msgs/GetControlSpace")
(setf (get dsr_msgs::GetControlSpace :request) dsr_msgs::GetControlSpaceRequest)
(setf (get dsr_msgs::GetControlSpace :response) dsr_msgs::GetControlSpaceResponse)

(defmethod dsr_msgs::GetControlSpaceRequest
  (:response () (instance dsr_msgs::GetControlSpaceResponse :init)))

(setf (get dsr_msgs::GetControlSpaceRequest :md5sum-) "1a5780af2202e44900c27ba310bfe10c")
(setf (get dsr_msgs::GetControlSpaceRequest :datatype-) "dsr_msgs/GetControlSpaceRequest")
(setf (get dsr_msgs::GetControlSpaceRequest :definition-)
      "#____________________________________________________________________________________________
# get_control_space()  
#____________________________________________________________________________________________
# This service returns the current control space.

---
int8    space        # Control mode : Joint space control(1), Task space control(2)
bool    success

")

(setf (get dsr_msgs::GetControlSpaceResponse :md5sum-) "1a5780af2202e44900c27ba310bfe10c")
(setf (get dsr_msgs::GetControlSpaceResponse :datatype-) "dsr_msgs/GetControlSpaceResponse")
(setf (get dsr_msgs::GetControlSpaceResponse :definition-)
      "#____________________________________________________________________________________________
# get_control_space()  
#____________________________________________________________________________________________
# This service returns the current control space.

---
int8    space        # Control mode : Joint space control(1), Task space control(2)
bool    success

")



(provide :dsr_msgs/GetControlSpace "1a5780af2202e44900c27ba310bfe10c")


