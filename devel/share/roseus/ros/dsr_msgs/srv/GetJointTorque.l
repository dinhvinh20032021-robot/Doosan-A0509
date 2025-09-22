;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetJointTorque)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetJointTorque (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETJOINTTORQUE")
  (make-package "DSR_MSGS::GETJOINTTORQUE"))
(unless (find-package "DSR_MSGS::GETJOINTTORQUEREQUEST")
  (make-package "DSR_MSGS::GETJOINTTORQUEREQUEST"))
(unless (find-package "DSR_MSGS::GETJOINTTORQUERESPONSE")
  (make-package "DSR_MSGS::GETJOINTTORQUERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetJointTorqueRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetJointTorqueRequest
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

(defclass dsr_msgs::GetJointTorqueResponse
  :super ros::object
  :slots (_jts _success ))

(defmethod dsr_msgs::GetJointTorqueResponse
  (:init
   (&key
    ((:jts __jts) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _jts __jts)
   (setq _success __success)
   self)
  (:jts
   (&optional __jts)
   (if __jts (setq _jts __jts)) _jts)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float64[6] _jts
    (* 8    6)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _jts
     (dotimes (i 6)
       (sys::poke (elt _jts i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _jts
   (dotimes (i (length _jts))
     (setf (elt _jts i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetJointTorque
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetJointTorque :md5sum-) "d2bba428be693e219072ca1b9c1a000a")
(setf (get dsr_msgs::GetJointTorque :datatype-) "dsr_msgs/GetJointTorque")
(setf (get dsr_msgs::GetJointTorque :request) dsr_msgs::GetJointTorqueRequest)
(setf (get dsr_msgs::GetJointTorque :response) dsr_msgs::GetJointTorqueResponse)

(defmethod dsr_msgs::GetJointTorqueRequest
  (:response () (instance dsr_msgs::GetJointTorqueResponse :init)))

(setf (get dsr_msgs::GetJointTorqueRequest :md5sum-) "d2bba428be693e219072ca1b9c1a000a")
(setf (get dsr_msgs::GetJointTorqueRequest :datatype-) "dsr_msgs/GetJointTorqueRequest")
(setf (get dsr_msgs::GetJointTorqueRequest :definition-)
      "#____________________________________________________________________________________________
# get_joint_torque()
# returns the sensor torque value of the current joint.
#____________________________________________________________________________________________
# This service returns the sensor torque value of the current joint.

---
float64[6] jts         # value of JTS(Joint Torque Sensor) 
bool       success
")

(setf (get dsr_msgs::GetJointTorqueResponse :md5sum-) "d2bba428be693e219072ca1b9c1a000a")
(setf (get dsr_msgs::GetJointTorqueResponse :datatype-) "dsr_msgs/GetJointTorqueResponse")
(setf (get dsr_msgs::GetJointTorqueResponse :definition-)
      "#____________________________________________________________________________________________
# get_joint_torque()
# returns the sensor torque value of the current joint.
#____________________________________________________________________________________________
# This service returns the sensor torque value of the current joint.

---
float64[6] jts         # value of JTS(Joint Torque Sensor) 
bool       success
")



(provide :dsr_msgs/GetJointTorque "d2bba428be693e219072ca1b9c1a000a")


