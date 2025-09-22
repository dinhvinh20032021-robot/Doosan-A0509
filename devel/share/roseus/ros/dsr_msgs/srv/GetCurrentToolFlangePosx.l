;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetCurrentToolFlangePosx)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetCurrentToolFlangePosx (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETCURRENTTOOLFLANGEPOSX")
  (make-package "DSR_MSGS::GETCURRENTTOOLFLANGEPOSX"))
(unless (find-package "DSR_MSGS::GETCURRENTTOOLFLANGEPOSXREQUEST")
  (make-package "DSR_MSGS::GETCURRENTTOOLFLANGEPOSXREQUEST"))
(unless (find-package "DSR_MSGS::GETCURRENTTOOLFLANGEPOSXRESPONSE")
  (make-package "DSR_MSGS::GETCURRENTTOOLFLANGEPOSXRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetCurrentToolFlangePosxRequest
  :super ros::object
  :slots (_ref ))

(defmethod dsr_msgs::GetCurrentToolFlangePosxRequest
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

(defclass dsr_msgs::GetCurrentToolFlangePosxResponse
  :super ros::object
  :slots (_pos _success ))

(defmethod dsr_msgs::GetCurrentToolFlangePosxResponse
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

(defclass dsr_msgs::GetCurrentToolFlangePosx
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetCurrentToolFlangePosx :md5sum-) "88547bdd1d2b717be93c04de7f32b5ea")
(setf (get dsr_msgs::GetCurrentToolFlangePosx :datatype-) "dsr_msgs/GetCurrentToolFlangePosx")
(setf (get dsr_msgs::GetCurrentToolFlangePosx :request) dsr_msgs::GetCurrentToolFlangePosxRequest)
(setf (get dsr_msgs::GetCurrentToolFlangePosx :response) dsr_msgs::GetCurrentToolFlangePosxResponse)

(defmethod dsr_msgs::GetCurrentToolFlangePosxRequest
  (:response () (instance dsr_msgs::GetCurrentToolFlangePosxResponse :init)))

(setf (get dsr_msgs::GetCurrentToolFlangePosxRequest :md5sum-) "88547bdd1d2b717be93c04de7f32b5ea")
(setf (get dsr_msgs::GetCurrentToolFlangePosxRequest :datatype-) "dsr_msgs/GetCurrentToolFlangePosxRequest")
(setf (get dsr_msgs::GetCurrentToolFlangePosxRequest :definition-)
      "#____________________________________________________________________________________________
# get_current_tool_flange_posx()  
#____________________________________________________________________________________________
# This service returns the pose of the current tool flange based on the ref coordinate. In other words, it means the return to tcp=(0,0,0,0,0,0).

int8        ref               # DR_BASE(0), DR_WORLD(2)
---
float64[6]  pos               # Pose of tool flange(posx) 
bool        success
")

(setf (get dsr_msgs::GetCurrentToolFlangePosxResponse :md5sum-) "88547bdd1d2b717be93c04de7f32b5ea")
(setf (get dsr_msgs::GetCurrentToolFlangePosxResponse :datatype-) "dsr_msgs/GetCurrentToolFlangePosxResponse")
(setf (get dsr_msgs::GetCurrentToolFlangePosxResponse :definition-)
      "#____________________________________________________________________________________________
# get_current_tool_flange_posx()  
#____________________________________________________________________________________________
# This service returns the pose of the current tool flange based on the ref coordinate. In other words, it means the return to tcp=(0,0,0,0,0,0).

int8        ref               # DR_BASE(0), DR_WORLD(2)
---
float64[6]  pos               # Pose of tool flange(posx) 
bool        success
")



(provide :dsr_msgs/GetCurrentToolFlangePosx "88547bdd1d2b717be93c04de7f32b5ea")


