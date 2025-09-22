;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetDesiredVelx)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetDesiredVelx (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETDESIREDVELX")
  (make-package "DSR_MSGS::GETDESIREDVELX"))
(unless (find-package "DSR_MSGS::GETDESIREDVELXREQUEST")
  (make-package "DSR_MSGS::GETDESIREDVELXREQUEST"))
(unless (find-package "DSR_MSGS::GETDESIREDVELXRESPONSE")
  (make-package "DSR_MSGS::GETDESIREDVELXRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetDesiredVelxRequest
  :super ros::object
  :slots (_ref ))

(defmethod dsr_msgs::GetDesiredVelxRequest
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

(defclass dsr_msgs::GetDesiredVelxResponse
  :super ros::object
  :slots (_vel _success ))

(defmethod dsr_msgs::GetDesiredVelxResponse
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

(defclass dsr_msgs::GetDesiredVelx
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetDesiredVelx :md5sum-) "1ccbebbf66c48f74afb258f026a6daff")
(setf (get dsr_msgs::GetDesiredVelx :datatype-) "dsr_msgs/GetDesiredVelx")
(setf (get dsr_msgs::GetDesiredVelx :request) dsr_msgs::GetDesiredVelxRequest)
(setf (get dsr_msgs::GetDesiredVelx :response) dsr_msgs::GetDesiredVelxResponse)

(defmethod dsr_msgs::GetDesiredVelxRequest
  (:response () (instance dsr_msgs::GetDesiredVelxResponse :init)))

(setf (get dsr_msgs::GetDesiredVelxRequest :md5sum-) "1ccbebbf66c48f74afb258f026a6daff")
(setf (get dsr_msgs::GetDesiredVelxRequest :datatype-) "dsr_msgs/GetDesiredVelxRequest")
(setf (get dsr_msgs::GetDesiredVelxRequest :definition-)
      "#____________________________________________________________________________________________
# get_desired_velx(ref)  
#____________________________________________________________________________________________
# This service returns the target velocity of the current tool based on the ref coordinate. 
# It cannot be used in the movej, movejx, or movesj service.

int8       ref               # DR_BASE(0), DR_WORLD(2)
---
float64[6] vel               # Tool velocity
bool       success
")

(setf (get dsr_msgs::GetDesiredVelxResponse :md5sum-) "1ccbebbf66c48f74afb258f026a6daff")
(setf (get dsr_msgs::GetDesiredVelxResponse :datatype-) "dsr_msgs/GetDesiredVelxResponse")
(setf (get dsr_msgs::GetDesiredVelxResponse :definition-)
      "#____________________________________________________________________________________________
# get_desired_velx(ref)  
#____________________________________________________________________________________________
# This service returns the target velocity of the current tool based on the ref coordinate. 
# It cannot be used in the movej, movejx, or movesj service.

int8       ref               # DR_BASE(0), DR_WORLD(2)
---
float64[6] vel               # Tool velocity
bool       success
")



(provide :dsr_msgs/GetDesiredVelx "1ccbebbf66c48f74afb258f026a6daff")


