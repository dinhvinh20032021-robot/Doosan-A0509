;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetToolForce)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetToolForce (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETTOOLFORCE")
  (make-package "DSR_MSGS::GETTOOLFORCE"))
(unless (find-package "DSR_MSGS::GETTOOLFORCEREQUEST")
  (make-package "DSR_MSGS::GETTOOLFORCEREQUEST"))
(unless (find-package "DSR_MSGS::GETTOOLFORCERESPONSE")
  (make-package "DSR_MSGS::GETTOOLFORCERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetToolForceRequest
  :super ros::object
  :slots (_ref ))

(defmethod dsr_msgs::GetToolForceRequest
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

(defclass dsr_msgs::GetToolForceResponse
  :super ros::object
  :slots (_tool_force _success ))

(defmethod dsr_msgs::GetToolForceResponse
  (:init
   (&key
    ((:tool_force __tool_force) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _tool_force __tool_force)
   (setq _success __success)
   self)
  (:tool_force
   (&optional __tool_force)
   (if __tool_force (setq _tool_force __tool_force)) _tool_force)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float64[6] _tool_force
    (* 8    6)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _tool_force
     (dotimes (i 6)
       (sys::poke (elt _tool_force i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _tool_force
   (dotimes (i (length _tool_force))
     (setf (elt _tool_force i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetToolForce
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetToolForce :md5sum-) "9590f477df24281be1fec298a858c77f")
(setf (get dsr_msgs::GetToolForce :datatype-) "dsr_msgs/GetToolForce")
(setf (get dsr_msgs::GetToolForce :request) dsr_msgs::GetToolForceRequest)
(setf (get dsr_msgs::GetToolForce :response) dsr_msgs::GetToolForceResponse)

(defmethod dsr_msgs::GetToolForceRequest
  (:response () (instance dsr_msgs::GetToolForceResponse :init)))

(setf (get dsr_msgs::GetToolForceRequest :md5sum-) "9590f477df24281be1fec298a858c77f")
(setf (get dsr_msgs::GetToolForceRequest :datatype-) "dsr_msgs/GetToolForceRequest")
(setf (get dsr_msgs::GetToolForceRequest :definition-)
      "#____________________________________________________________________________________________
# get_tool_force(ref)
# returns the external force applied to the current tool
#____________________________________________________________________________________________
# This service returns the external force applied to the current tool based on the ref coordinate. 
# The force is based on the base coordinate while the moment is based on the tool coordinate.

int8       ref               # DR_BASE(0), DR_WORLD(2)
---
float64[6] tool_force        # External force applied to the tool
bool       success
")

(setf (get dsr_msgs::GetToolForceResponse :md5sum-) "9590f477df24281be1fec298a858c77f")
(setf (get dsr_msgs::GetToolForceResponse :datatype-) "dsr_msgs/GetToolForceResponse")
(setf (get dsr_msgs::GetToolForceResponse :definition-)
      "#____________________________________________________________________________________________
# get_tool_force(ref)
# returns the external force applied to the current tool
#____________________________________________________________________________________________
# This service returns the external force applied to the current tool based on the ref coordinate. 
# The force is based on the base coordinate while the moment is based on the tool coordinate.

int8       ref               # DR_BASE(0), DR_WORLD(2)
---
float64[6] tool_force        # External force applied to the tool
bool       success
")



(provide :dsr_msgs/GetToolForce "9590f477df24281be1fec298a858c77f")


