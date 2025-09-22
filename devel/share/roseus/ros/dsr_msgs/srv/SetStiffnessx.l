;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetStiffnessx)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetStiffnessx (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETSTIFFNESSX")
  (make-package "DSR_MSGS::SETSTIFFNESSX"))
(unless (find-package "DSR_MSGS::SETSTIFFNESSXREQUEST")
  (make-package "DSR_MSGS::SETSTIFFNESSXREQUEST"))
(unless (find-package "DSR_MSGS::SETSTIFFNESSXRESPONSE")
  (make-package "DSR_MSGS::SETSTIFFNESSXRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetStiffnessxRequest
  :super ros::object
  :slots (_stx _ref _time ))

(defmethod dsr_msgs::SetStiffnessxRequest
  (:init
   (&key
    ((:stx __stx) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:ref __ref) 0)
    ((:time __time) 0.0)
    )
   (send-super :init)
   (setq _stx __stx)
   (setq _ref (round __ref))
   (setq _time (float __time))
   self)
  (:stx
   (&optional __stx)
   (if __stx (setq _stx __stx)) _stx)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; float64[6] _stx
    (* 8    6)
    ;; int8 _ref
    1
    ;; float64 _time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _stx
     (dotimes (i 6)
       (sys::poke (elt _stx i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8 _ref
       (write-byte _ref s)
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _stx
   (dotimes (i (length _stx))
     (setf (elt _stx i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass dsr_msgs::SetStiffnessxResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetStiffnessxResponse
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

(defclass dsr_msgs::SetStiffnessx
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetStiffnessx :md5sum-) "13f8b8d245232e98ce000bd4e6262bcd")
(setf (get dsr_msgs::SetStiffnessx :datatype-) "dsr_msgs/SetStiffnessx")
(setf (get dsr_msgs::SetStiffnessx :request) dsr_msgs::SetStiffnessxRequest)
(setf (get dsr_msgs::SetStiffnessx :response) dsr_msgs::SetStiffnessxResponse)

(defmethod dsr_msgs::SetStiffnessxRequest
  (:response () (instance dsr_msgs::SetStiffnessxResponse :init)))

(setf (get dsr_msgs::SetStiffnessxRequest :md5sum-) "13f8b8d245232e98ce000bd4e6262bcd")
(setf (get dsr_msgs::SetStiffnessxRequest :datatype-) "dsr_msgs/SetStiffnessxRequest")
(setf (get dsr_msgs::SetStiffnessxRequest :definition-)
      "#____________________________________________________________________________________________
# set_stiffnessx  
#____________________________________________________________________________________________

float64[6] stx               # default[500, 500, 500, 100, 100, 100], Three translational stiffnesses + Three rotational stiffnesses
int8       ref               # the preset reference coordinate system.
float64    time              # Stiffness varying time(0 ~ 1.0) [sec], Linear transition during the specified time   
---
bool       success 
")

(setf (get dsr_msgs::SetStiffnessxResponse :md5sum-) "13f8b8d245232e98ce000bd4e6262bcd")
(setf (get dsr_msgs::SetStiffnessxResponse :datatype-) "dsr_msgs/SetStiffnessxResponse")
(setf (get dsr_msgs::SetStiffnessxResponse :definition-)
      "#____________________________________________________________________________________________
# set_stiffnessx  
#____________________________________________________________________________________________

float64[6] stx               # default[500, 500, 500, 100, 100, 100], Three translational stiffnesses + Three rotational stiffnesses
int8       ref               # the preset reference coordinate system.
float64    time              # Stiffness varying time(0 ~ 1.0) [sec], Linear transition during the specified time   
---
bool       success 
")



(provide :dsr_msgs/SetStiffnessx "13f8b8d245232e98ce000bd4e6262bcd")


