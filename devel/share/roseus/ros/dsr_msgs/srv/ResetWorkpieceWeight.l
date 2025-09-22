;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ResetWorkpieceWeight)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ResetWorkpieceWeight (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::RESETWORKPIECEWEIGHT")
  (make-package "DSR_MSGS::RESETWORKPIECEWEIGHT"))
(unless (find-package "DSR_MSGS::RESETWORKPIECEWEIGHTREQUEST")
  (make-package "DSR_MSGS::RESETWORKPIECEWEIGHTREQUEST"))
(unless (find-package "DSR_MSGS::RESETWORKPIECEWEIGHTRESPONSE")
  (make-package "DSR_MSGS::RESETWORKPIECEWEIGHTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::ResetWorkpieceWeightRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::ResetWorkpieceWeightRequest
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

(defclass dsr_msgs::ResetWorkpieceWeightResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::ResetWorkpieceWeightResponse
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

(defclass dsr_msgs::ResetWorkpieceWeight
  :super ros::object
  :slots ())

(setf (get dsr_msgs::ResetWorkpieceWeight :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::ResetWorkpieceWeight :datatype-) "dsr_msgs/ResetWorkpieceWeight")
(setf (get dsr_msgs::ResetWorkpieceWeight :request) dsr_msgs::ResetWorkpieceWeightRequest)
(setf (get dsr_msgs::ResetWorkpieceWeight :response) dsr_msgs::ResetWorkpieceWeightResponse)

(defmethod dsr_msgs::ResetWorkpieceWeightRequest
  (:response () (instance dsr_msgs::ResetWorkpieceWeightResponse :init)))

(setf (get dsr_msgs::ResetWorkpieceWeightRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::ResetWorkpieceWeightRequest :datatype-) "dsr_msgs/ResetWorkpieceWeightRequest")
(setf (get dsr_msgs::ResetWorkpieceWeightRequest :definition-)
      "#____________________________________________________________________________________________
# reset_workpiece_weight
# Initializes the weight data of the material to initialize the algorithm before measuring the weight of the material.
# ____________________________________________________________________________________________

---
bool       success 
")

(setf (get dsr_msgs::ResetWorkpieceWeightResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::ResetWorkpieceWeightResponse :datatype-) "dsr_msgs/ResetWorkpieceWeightResponse")
(setf (get dsr_msgs::ResetWorkpieceWeightResponse :definition-)
      "#____________________________________________________________________________________________
# reset_workpiece_weight
# Initializes the weight data of the material to initialize the algorithm before measuring the weight of the material.
# ____________________________________________________________________________________________

---
bool       success 
")



(provide :dsr_msgs/ResetWorkpieceWeight "358e233cde0c8a8bcfea4ce193f8fc15")


