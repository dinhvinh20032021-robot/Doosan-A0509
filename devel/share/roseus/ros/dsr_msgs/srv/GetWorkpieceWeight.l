;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetWorkpieceWeight)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetWorkpieceWeight (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETWORKPIECEWEIGHT")
  (make-package "DSR_MSGS::GETWORKPIECEWEIGHT"))
(unless (find-package "DSR_MSGS::GETWORKPIECEWEIGHTREQUEST")
  (make-package "DSR_MSGS::GETWORKPIECEWEIGHTREQUEST"))
(unless (find-package "DSR_MSGS::GETWORKPIECEWEIGHTRESPONSE")
  (make-package "DSR_MSGS::GETWORKPIECEWEIGHTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetWorkpieceWeightRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetWorkpieceWeightRequest
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

(defclass dsr_msgs::GetWorkpieceWeightResponse
  :super ros::object
  :slots (_weight _success ))

(defmethod dsr_msgs::GetWorkpieceWeightResponse
  (:init
   (&key
    ((:weight __weight) 0.0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _weight (float __weight))
   (setq _success __success)
   self)
  (:weight
   (&optional __weight)
   (if __weight (setq _weight __weight)) _weight)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float32 _weight
    4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _weight
       (sys::poke _weight (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _weight
     (setq _weight (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetWorkpieceWeight
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetWorkpieceWeight :md5sum-) "79fbe0854a3099a7d5008b26f92ad15a")
(setf (get dsr_msgs::GetWorkpieceWeight :datatype-) "dsr_msgs/GetWorkpieceWeight")
(setf (get dsr_msgs::GetWorkpieceWeight :request) dsr_msgs::GetWorkpieceWeightRequest)
(setf (get dsr_msgs::GetWorkpieceWeight :response) dsr_msgs::GetWorkpieceWeightResponse)

(defmethod dsr_msgs::GetWorkpieceWeightRequest
  (:response () (instance dsr_msgs::GetWorkpieceWeightResponse :init)))

(setf (get dsr_msgs::GetWorkpieceWeightRequest :md5sum-) "79fbe0854a3099a7d5008b26f92ad15a")
(setf (get dsr_msgs::GetWorkpieceWeightRequest :datatype-) "dsr_msgs/GetWorkpieceWeightRequest")
(setf (get dsr_msgs::GetWorkpieceWeightRequest :definition-)
      "#____________________________________________________________________________________________
# get_workpiece_weight
#____________________________________________________________________________________________

---
float32       weight               # Measured weight, Negative value if error
bool          success
")

(setf (get dsr_msgs::GetWorkpieceWeightResponse :md5sum-) "79fbe0854a3099a7d5008b26f92ad15a")
(setf (get dsr_msgs::GetWorkpieceWeightResponse :datatype-) "dsr_msgs/GetWorkpieceWeightResponse")
(setf (get dsr_msgs::GetWorkpieceWeightResponse :definition-)
      "#____________________________________________________________________________________________
# get_workpiece_weight
#____________________________________________________________________________________________

---
float32       weight               # Measured weight, Negative value if error
bool          success
")



(provide :dsr_msgs/GetWorkpieceWeight "79fbe0854a3099a7d5008b26f92ad15a")


