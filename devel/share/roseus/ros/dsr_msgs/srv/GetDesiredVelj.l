;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetDesiredVelj)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetDesiredVelj (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETDESIREDVELJ")
  (make-package "DSR_MSGS::GETDESIREDVELJ"))
(unless (find-package "DSR_MSGS::GETDESIREDVELJREQUEST")
  (make-package "DSR_MSGS::GETDESIREDVELJREQUEST"))
(unless (find-package "DSR_MSGS::GETDESIREDVELJRESPONSE")
  (make-package "DSR_MSGS::GETDESIREDVELJRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetDesiredVeljRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetDesiredVeljRequest
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

(defclass dsr_msgs::GetDesiredVeljResponse
  :super ros::object
  :slots (_joint_vel _success ))

(defmethod dsr_msgs::GetDesiredVeljResponse
  (:init
   (&key
    ((:joint_vel __joint_vel) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _joint_vel __joint_vel)
   (setq _success __success)
   self)
  (:joint_vel
   (&optional __joint_vel)
   (if __joint_vel (setq _joint_vel __joint_vel)) _joint_vel)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float64[6] _joint_vel
    (* 8    6)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _joint_vel
     (dotimes (i 6)
       (sys::poke (elt _joint_vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _joint_vel
   (dotimes (i (length _joint_vel))
     (setf (elt _joint_vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetDesiredVelj
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetDesiredVelj :md5sum-) "88bf0c6c29b23cd493e33b3354d4f21a")
(setf (get dsr_msgs::GetDesiredVelj :datatype-) "dsr_msgs/GetDesiredVelj")
(setf (get dsr_msgs::GetDesiredVelj :request) dsr_msgs::GetDesiredVeljRequest)
(setf (get dsr_msgs::GetDesiredVelj :response) dsr_msgs::GetDesiredVeljResponse)

(defmethod dsr_msgs::GetDesiredVeljRequest
  (:response () (instance dsr_msgs::GetDesiredVeljResponse :init)))

(setf (get dsr_msgs::GetDesiredVeljRequest :md5sum-) "88bf0c6c29b23cd493e33b3354d4f21a")
(setf (get dsr_msgs::GetDesiredVeljRequest :datatype-) "dsr_msgs/GetDesiredVeljRequest")
(setf (get dsr_msgs::GetDesiredVeljRequest :definition-)
      "#____________________________________________________________________________________________
# get_desired_velj()  
#____________________________________________________________________________________________

---
float64[6] joint_vel               # Target joint velocity 
bool       success
")

(setf (get dsr_msgs::GetDesiredVeljResponse :md5sum-) "88bf0c6c29b23cd493e33b3354d4f21a")
(setf (get dsr_msgs::GetDesiredVeljResponse :datatype-) "dsr_msgs/GetDesiredVeljResponse")
(setf (get dsr_msgs::GetDesiredVeljResponse :definition-)
      "#____________________________________________________________________________________________
# get_desired_velj()  
#____________________________________________________________________________________________

---
float64[6] joint_vel               # Target joint velocity 
bool       success
")



(provide :dsr_msgs/GetDesiredVelj "88bf0c6c29b23cd493e33b3354d4f21a")


