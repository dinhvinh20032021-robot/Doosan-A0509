;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::CheckForceCondition)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'CheckForceCondition (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::CHECKFORCECONDITION")
  (make-package "DSR_MSGS::CHECKFORCECONDITION"))
(unless (find-package "DSR_MSGS::CHECKFORCECONDITIONREQUEST")
  (make-package "DSR_MSGS::CHECKFORCECONDITIONREQUEST"))
(unless (find-package "DSR_MSGS::CHECKFORCECONDITIONRESPONSE")
  (make-package "DSR_MSGS::CHECKFORCECONDITIONRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::CheckForceConditionRequest
  :super ros::object
  :slots (_axis _min _max _ref ))

(defmethod dsr_msgs::CheckForceConditionRequest
  (:init
   (&key
    ((:axis __axis) 0)
    ((:min __min) 0.0)
    ((:max __max) 0.0)
    ((:ref __ref) 0)
    )
   (send-super :init)
   (setq _axis (round __axis))
   (setq _min (float __min))
   (setq _max (float __max))
   (setq _ref (round __ref))
   self)
  (:axis
   (&optional __axis)
   (if __axis (setq _axis __axis)) _axis)
  (:min
   (&optional __min)
   (if __min (setq _min __min)) _min)
  (:max
   (&optional __max)
   (if __max (setq _max __max)) _max)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:serialization-length
   ()
   (+
    ;; int8 _axis
    1
    ;; float64 _min
    8
    ;; float64 _max
    8
    ;; int8 _ref
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _axis
       (write-byte _axis s)
     ;; float64 _min
       (sys::poke _min (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max
       (sys::poke _max (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8 _ref
       (write-byte _ref s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _axis
     (setq _axis (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _axis 127) (setq _axis (- _axis 256)))
   ;; float64 _min
     (setq _min (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max
     (setq _max (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;;
   self)
  )

(defclass dsr_msgs::CheckForceConditionResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::CheckForceConditionResponse
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

(defclass dsr_msgs::CheckForceCondition
  :super ros::object
  :slots ())

(setf (get dsr_msgs::CheckForceCondition :md5sum-) "4904883ec6cc91634eacedbab3722558")
(setf (get dsr_msgs::CheckForceCondition :datatype-) "dsr_msgs/CheckForceCondition")
(setf (get dsr_msgs::CheckForceCondition :request) dsr_msgs::CheckForceConditionRequest)
(setf (get dsr_msgs::CheckForceCondition :response) dsr_msgs::CheckForceConditionResponse)

(defmethod dsr_msgs::CheckForceConditionRequest
  (:response () (instance dsr_msgs::CheckForceConditionResponse :init)))

(setf (get dsr_msgs::CheckForceConditionRequest :md5sum-) "4904883ec6cc91634eacedbab3722558")
(setf (get dsr_msgs::CheckForceConditionRequest :datatype-) "dsr_msgs/CheckForceConditionRequest")
(setf (get dsr_msgs::CheckForceConditionRequest :definition-)
      "#____________________________________________________________________________________________
# check_force_condition 
#This service checks the status of the given force. It disregards the force direction and only compares the sizes. 
#This condition can be repeated with the while or if statement. Measuring the force, axis is based on the ref coordinate and measuring the moment,
#axis is based on the tool coordinate.
#____________________________________________________________________________________________

int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2), DR_AXIS_A(10), DR_AXIS_B(11), DR_AXIS_C(12) 
float64    min               # min >=0.0   
float64    max               # max >=0.0 
int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200)
                             # <DR_WORLD is only available in M2.40 or later> 
---
bool       success                 # True or False

")

(setf (get dsr_msgs::CheckForceConditionResponse :md5sum-) "4904883ec6cc91634eacedbab3722558")
(setf (get dsr_msgs::CheckForceConditionResponse :datatype-) "dsr_msgs/CheckForceConditionResponse")
(setf (get dsr_msgs::CheckForceConditionResponse :definition-)
      "#____________________________________________________________________________________________
# check_force_condition 
#This service checks the status of the given force. It disregards the force direction and only compares the sizes. 
#This condition can be repeated with the while or if statement. Measuring the force, axis is based on the ref coordinate and measuring the moment,
#axis is based on the tool coordinate.
#____________________________________________________________________________________________

int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2), DR_AXIS_A(10), DR_AXIS_B(11), DR_AXIS_C(12) 
float64    min               # min >=0.0   
float64    max               # max >=0.0 
int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200)
                             # <DR_WORLD is only available in M2.40 or later> 
---
bool       success                 # True or False

")



(provide :dsr_msgs/CheckForceCondition "4904883ec6cc91634eacedbab3722558")


