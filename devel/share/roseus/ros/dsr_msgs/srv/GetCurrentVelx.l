;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetCurrentVelx)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetCurrentVelx (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETCURRENTVELX")
  (make-package "DSR_MSGS::GETCURRENTVELX"))
(unless (find-package "DSR_MSGS::GETCURRENTVELXREQUEST")
  (make-package "DSR_MSGS::GETCURRENTVELXREQUEST"))
(unless (find-package "DSR_MSGS::GETCURRENTVELXRESPONSE")
  (make-package "DSR_MSGS::GETCURRENTVELXRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetCurrentVelxRequest
  :super ros::object
  :slots (_ref ))

(defmethod dsr_msgs::GetCurrentVelxRequest
  (:init
   (&key
    ((:ref __ref) 0)
    )
   (send-super :init)
   (setq _ref (round __ref))
   self)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:serialization-length
   ()
   (+
    ;; int8 _ref
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _ref
       (write-byte _ref s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;;
   self)
  )

(defclass dsr_msgs::GetCurrentVelxResponse
  :super ros::object
  :slots (_vel _success ))

(defmethod dsr_msgs::GetCurrentVelxResponse
  (:init
   (&key
    ((:vel __vel) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _vel __vel)
   (setq _success __success)
   self)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float64[6] _vel
    (* 8    6)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _vel
     (dotimes (i 6)
       (sys::poke (elt _vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _vel
   (dotimes (i (length _vel))
     (setf (elt _vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetCurrentVelx
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetCurrentVelx :md5sum-) "1ccbebbf66c48f74afb258f026a6daff")
(setf (get dsr_msgs::GetCurrentVelx :datatype-) "dsr_msgs/GetCurrentVelx")
(setf (get dsr_msgs::GetCurrentVelx :request) dsr_msgs::GetCurrentVelxRequest)
(setf (get dsr_msgs::GetCurrentVelx :response) dsr_msgs::GetCurrentVelxResponse)

(defmethod dsr_msgs::GetCurrentVelxRequest
  (:response () (instance dsr_msgs::GetCurrentVelxResponse :init)))

(setf (get dsr_msgs::GetCurrentVelxRequest :md5sum-) "1ccbebbf66c48f74afb258f026a6daff")
(setf (get dsr_msgs::GetCurrentVelxRequest :datatype-) "dsr_msgs/GetCurrentVelxRequest")
(setf (get dsr_msgs::GetCurrentVelxRequest :definition-)
      "#____________________________________________________________________________________________
# get_current_velx(ref)  
#____________________________________________________________________________________________
# This service returns the current tool velocity based on the ref coordinate.

int8       ref               # DR_BASE(0), DR_WORLD(2)
---
float64[6] vel               # Tool velocity
bool       success
")

(setf (get dsr_msgs::GetCurrentVelxResponse :md5sum-) "1ccbebbf66c48f74afb258f026a6daff")
(setf (get dsr_msgs::GetCurrentVelxResponse :datatype-) "dsr_msgs/GetCurrentVelxResponse")
(setf (get dsr_msgs::GetCurrentVelxResponse :definition-)
      "#____________________________________________________________________________________________
# get_current_velx(ref)  
#____________________________________________________________________________________________
# This service returns the current tool velocity based on the ref coordinate.

int8       ref               # DR_BASE(0), DR_WORLD(2)
---
float64[6] vel               # Tool velocity
bool       success
")



(provide :dsr_msgs/GetCurrentVelx "1ccbebbf66c48f74afb258f026a6daff")


