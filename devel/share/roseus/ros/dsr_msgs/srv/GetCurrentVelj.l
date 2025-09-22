;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetCurrentVelj)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetCurrentVelj (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETCURRENTVELJ")
  (make-package "DSR_MSGS::GETCURRENTVELJ"))
(unless (find-package "DSR_MSGS::GETCURRENTVELJREQUEST")
  (make-package "DSR_MSGS::GETCURRENTVELJREQUEST"))
(unless (find-package "DSR_MSGS::GETCURRENTVELJRESPONSE")
  (make-package "DSR_MSGS::GETCURRENTVELJRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetCurrentVeljRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetCurrentVeljRequest
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

(defclass dsr_msgs::GetCurrentVeljResponse
  :super ros::object
  :slots (_joint_speed _success ))

(defmethod dsr_msgs::GetCurrentVeljResponse
  (:init
   (&key
    ((:joint_speed __joint_speed) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _joint_speed __joint_speed)
   (setq _success __success)
   self)
  (:joint_speed
   (&optional __joint_speed)
   (if __joint_speed (setq _joint_speed __joint_speed)) _joint_speed)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float64[6] _joint_speed
    (* 8    6)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _joint_speed
     (dotimes (i 6)
       (sys::poke (elt _joint_speed i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _joint_speed
   (dotimes (i (length _joint_speed))
     (setf (elt _joint_speed i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetCurrentVelj
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetCurrentVelj :md5sum-) "9162ee4b17260f95d8a41690c4c86ea1")
(setf (get dsr_msgs::GetCurrentVelj :datatype-) "dsr_msgs/GetCurrentVelj")
(setf (get dsr_msgs::GetCurrentVelj :request) dsr_msgs::GetCurrentVeljRequest)
(setf (get dsr_msgs::GetCurrentVelj :response) dsr_msgs::GetCurrentVeljResponse)

(defmethod dsr_msgs::GetCurrentVeljRequest
  (:response () (instance dsr_msgs::GetCurrentVeljResponse :init)))

(setf (get dsr_msgs::GetCurrentVeljRequest :md5sum-) "9162ee4b17260f95d8a41690c4c86ea1")
(setf (get dsr_msgs::GetCurrentVeljRequest :datatype-) "dsr_msgs/GetCurrentVeljRequest")
(setf (get dsr_msgs::GetCurrentVeljRequest :definition-)
      "#____________________________________________________________________________________________
# get_current_velj()  
#____________________________________________________________________________________________
# This service returns the current target joint velocity. It cannot be used in the movel, movec, movesx, moveb, move_spiral, or move_periodic command.

---
float64[6]  joint_speed               # joint speed 
bool        success
")

(setf (get dsr_msgs::GetCurrentVeljResponse :md5sum-) "9162ee4b17260f95d8a41690c4c86ea1")
(setf (get dsr_msgs::GetCurrentVeljResponse :datatype-) "dsr_msgs/GetCurrentVeljResponse")
(setf (get dsr_msgs::GetCurrentVeljResponse :definition-)
      "#____________________________________________________________________________________________
# get_current_velj()  
#____________________________________________________________________________________________
# This service returns the current target joint velocity. It cannot be used in the movel, movec, movesx, moveb, move_spiral, or move_periodic command.

---
float64[6]  joint_speed               # joint speed 
bool        success
")



(provide :dsr_msgs/GetCurrentVelj "9162ee4b17260f95d8a41690c4c86ea1")


