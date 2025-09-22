;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetDesiredPosx)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetDesiredPosx (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETDESIREDPOSX")
  (make-package "DSR_MSGS::GETDESIREDPOSX"))
(unless (find-package "DSR_MSGS::GETDESIREDPOSXREQUEST")
  (make-package "DSR_MSGS::GETDESIREDPOSXREQUEST"))
(unless (find-package "DSR_MSGS::GETDESIREDPOSXRESPONSE")
  (make-package "DSR_MSGS::GETDESIREDPOSXRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetDesiredPosxRequest
  :super ros::object
  :slots (_ref ))

(defmethod dsr_msgs::GetDesiredPosxRequest
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

(defclass dsr_msgs::GetDesiredPosxResponse
  :super ros::object
  :slots (_pos _success ))

(defmethod dsr_msgs::GetDesiredPosxResponse
  (:init
   (&key
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _success __success)
   self)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float64[6] _pos
    (* 8    6)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _pos
     (dotimes (i 6)
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetDesiredPosx
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetDesiredPosx :md5sum-) "88547bdd1d2b717be93c04de7f32b5ea")
(setf (get dsr_msgs::GetDesiredPosx :datatype-) "dsr_msgs/GetDesiredPosx")
(setf (get dsr_msgs::GetDesiredPosx :request) dsr_msgs::GetDesiredPosxRequest)
(setf (get dsr_msgs::GetDesiredPosx :response) dsr_msgs::GetDesiredPosxResponse)

(defmethod dsr_msgs::GetDesiredPosxRequest
  (:response () (instance dsr_msgs::GetDesiredPosxResponse :init)))

(setf (get dsr_msgs::GetDesiredPosxRequest :md5sum-) "88547bdd1d2b717be93c04de7f32b5ea")
(setf (get dsr_msgs::GetDesiredPosxRequest :datatype-) "dsr_msgs/GetDesiredPosxRequest")
(setf (get dsr_msgs::GetDesiredPosxRequest :definition-)
      "#____________________________________________________________________________________________
# get_desired_posx(ref)  
#____________________________________________________________________________________________
# This service returns the target pose of the current tool. The pose is based on the ref coordinate.

int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
---
float64[6] pos               # task pos(posx)
bool       success
")

(setf (get dsr_msgs::GetDesiredPosxResponse :md5sum-) "88547bdd1d2b717be93c04de7f32b5ea")
(setf (get dsr_msgs::GetDesiredPosxResponse :datatype-) "dsr_msgs/GetDesiredPosxResponse")
(setf (get dsr_msgs::GetDesiredPosxResponse :definition-)
      "#____________________________________________________________________________________________
# get_desired_posx(ref)  
#____________________________________________________________________________________________
# This service returns the target pose of the current tool. The pose is based on the ref coordinate.

int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
---
float64[6] pos               # task pos(posx)
bool       success
")



(provide :dsr_msgs/GetDesiredPosx "88547bdd1d2b717be93c04de7f32b5ea")


