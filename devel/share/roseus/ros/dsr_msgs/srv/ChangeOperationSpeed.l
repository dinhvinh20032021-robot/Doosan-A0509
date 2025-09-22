;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ChangeOperationSpeed)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ChangeOperationSpeed (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::CHANGEOPERATIONSPEED")
  (make-package "DSR_MSGS::CHANGEOPERATIONSPEED"))
(unless (find-package "DSR_MSGS::CHANGEOPERATIONSPEEDREQUEST")
  (make-package "DSR_MSGS::CHANGEOPERATIONSPEEDREQUEST"))
(unless (find-package "DSR_MSGS::CHANGEOPERATIONSPEEDRESPONSE")
  (make-package "DSR_MSGS::CHANGEOPERATIONSPEEDRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::ChangeOperationSpeedRequest
  :super ros::object
  :slots (_speed ))

(defmethod dsr_msgs::ChangeOperationSpeedRequest
  (:init
   (&key
    ((:speed __speed) 0)
    )
   (send-super :init)
   (setq _speed (round __speed))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; int8 _speed
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _speed
       (write-byte _speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _speed
     (setq _speed (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _speed 127) (setq _speed (- _speed 256)))
   ;;
   self)
  )

(defclass dsr_msgs::ChangeOperationSpeedResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::ChangeOperationSpeedResponse
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

(defclass dsr_msgs::ChangeOperationSpeed
  :super ros::object
  :slots ())

(setf (get dsr_msgs::ChangeOperationSpeed :md5sum-) "a79c96510dc9b2ea7b678458340a61bf")
(setf (get dsr_msgs::ChangeOperationSpeed :datatype-) "dsr_msgs/ChangeOperationSpeed")
(setf (get dsr_msgs::ChangeOperationSpeed :request) dsr_msgs::ChangeOperationSpeedRequest)
(setf (get dsr_msgs::ChangeOperationSpeed :response) dsr_msgs::ChangeOperationSpeedResponse)

(defmethod dsr_msgs::ChangeOperationSpeedRequest
  (:response () (instance dsr_msgs::ChangeOperationSpeedResponse :init)))

(setf (get dsr_msgs::ChangeOperationSpeedRequest :md5sum-) "a79c96510dc9b2ea7b678458340a61bf")
(setf (get dsr_msgs::ChangeOperationSpeedRequest :datatype-) "dsr_msgs/ChangeOperationSpeedRequest")
(setf (get dsr_msgs::ChangeOperationSpeedRequest :definition-)
      "#____________________________________________________________________________________________
# change_operation_speed
#____________________________________________________________________________________________

int8 speed              # operation speed: (1~100)
---
bool success

")

(setf (get dsr_msgs::ChangeOperationSpeedResponse :md5sum-) "a79c96510dc9b2ea7b678458340a61bf")
(setf (get dsr_msgs::ChangeOperationSpeedResponse :datatype-) "dsr_msgs/ChangeOperationSpeedResponse")
(setf (get dsr_msgs::ChangeOperationSpeedResponse :definition-)
      "#____________________________________________________________________________________________
# change_operation_speed
#____________________________________________________________________________________________

int8 speed              # operation speed: (1~100)
---
bool success

")



(provide :dsr_msgs/ChangeOperationSpeed "a79c96510dc9b2ea7b678458340a61bf")


