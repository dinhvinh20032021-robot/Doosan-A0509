;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::IsDoneBoltTightening)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'IsDoneBoltTightening (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::ISDONEBOLTTIGHTENING")
  (make-package "DSR_MSGS::ISDONEBOLTTIGHTENING"))
(unless (find-package "DSR_MSGS::ISDONEBOLTTIGHTENINGREQUEST")
  (make-package "DSR_MSGS::ISDONEBOLTTIGHTENINGREQUEST"))
(unless (find-package "DSR_MSGS::ISDONEBOLTTIGHTENINGRESPONSE")
  (make-package "DSR_MSGS::ISDONEBOLTTIGHTENINGRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::IsDoneBoltTighteningRequest
  :super ros::object
  :slots (_m _timeout _axis ))

(defmethod dsr_msgs::IsDoneBoltTighteningRequest
  (:init
   (&key
    ((:m __m) 0.0)
    ((:timeout __timeout) 0.0)
    ((:axis __axis) 0)
    )
   (send-super :init)
   (setq _m (float __m))
   (setq _timeout (float __timeout))
   (setq _axis (round __axis))
   self)
  (:m
   (&optional __m)
   (if __m (setq _m __m)) _m)
  (:timeout
   (&optional __timeout)
   (if __timeout (setq _timeout __timeout)) _timeout)
  (:axis
   (&optional __axis)
   (if __axis (setq _axis __axis)) _axis)
  (:serialization-length
   ()
   (+
    ;; float64 _m
    8
    ;; float64 _timeout
    8
    ;; int8 _axis
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _m
       (sys::poke _m (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _timeout
       (sys::poke _timeout (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8 _axis
       (write-byte _axis s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _m
     (setq _m (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _timeout
     (setq _timeout (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8 _axis
     (setq _axis (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _axis 127) (setq _axis (- _axis 256)))
   ;;
   self)
  )

(defclass dsr_msgs::IsDoneBoltTighteningResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::IsDoneBoltTighteningResponse
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

(defclass dsr_msgs::IsDoneBoltTightening
  :super ros::object
  :slots ())

(setf (get dsr_msgs::IsDoneBoltTightening :md5sum-) "be84affb8d741115ea457a393a21b8b5")
(setf (get dsr_msgs::IsDoneBoltTightening :datatype-) "dsr_msgs/IsDoneBoltTightening")
(setf (get dsr_msgs::IsDoneBoltTightening :request) dsr_msgs::IsDoneBoltTighteningRequest)
(setf (get dsr_msgs::IsDoneBoltTightening :response) dsr_msgs::IsDoneBoltTighteningResponse)

(defmethod dsr_msgs::IsDoneBoltTighteningRequest
  (:response () (instance dsr_msgs::IsDoneBoltTighteningResponse :init)))

(setf (get dsr_msgs::IsDoneBoltTighteningRequest :md5sum-) "be84affb8d741115ea457a393a21b8b5")
(setf (get dsr_msgs::IsDoneBoltTighteningRequest :datatype-) "dsr_msgs/IsDoneBoltTighteningRequest")
(setf (get dsr_msgs::IsDoneBoltTighteningRequest :definition-)
      "#____________________________________________________________________________________________
# is_done_bolt_tightening  
#____________________________________________________________________________________________

float64    m                 # Target torque  
float64    timeout           # Monitoring duration [sec]  
int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
---
bool       success
")

(setf (get dsr_msgs::IsDoneBoltTighteningResponse :md5sum-) "be84affb8d741115ea457a393a21b8b5")
(setf (get dsr_msgs::IsDoneBoltTighteningResponse :datatype-) "dsr_msgs/IsDoneBoltTighteningResponse")
(setf (get dsr_msgs::IsDoneBoltTighteningResponse :definition-)
      "#____________________________________________________________________________________________
# is_done_bolt_tightening  
#____________________________________________________________________________________________

float64    m                 # Target torque  
float64    timeout           # Monitoring duration [sec]  
int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
---
bool       success
")



(provide :dsr_msgs/IsDoneBoltTightening "be84affb8d741115ea457a393a21b8b5")


