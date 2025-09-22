;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ParallelAxis2)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ParallelAxis2 (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::PARALLELAXIS2")
  (make-package "DSR_MSGS::PARALLELAXIS2"))
(unless (find-package "DSR_MSGS::PARALLELAXIS2REQUEST")
  (make-package "DSR_MSGS::PARALLELAXIS2REQUEST"))
(unless (find-package "DSR_MSGS::PARALLELAXIS2RESPONSE")
  (make-package "DSR_MSGS::PARALLELAXIS2RESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::ParallelAxis2Request
  :super ros::object
  :slots (_vect _axis _ref ))

(defmethod dsr_msgs::ParallelAxis2Request
  (:init
   (&key
    ((:vect __vect) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:axis __axis) 0)
    ((:ref __ref) 0)
    )
   (send-super :init)
   (setq _vect __vect)
   (setq _axis (round __axis))
   (setq _ref (round __ref))
   self)
  (:vect
   (&optional __vect)
   (if __vect (setq _vect __vect)) _vect)
  (:axis
   (&optional __axis)
   (if __axis (setq _axis __axis)) _axis)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:serialization-length
   ()
   (+
    ;; float64[3] _vect
    (* 8    3)
    ;; int8 _axis
    1
    ;; int8 _ref
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[3] _vect
     (dotimes (i 3)
       (sys::poke (elt _vect i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8 _axis
       (write-byte _axis s)
     ;; int8 _ref
       (write-byte _ref s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[3] _vect
   (dotimes (i (length _vect))
     (setf (elt _vect i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int8 _axis
     (setq _axis (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _axis 127) (setq _axis (- _axis 256)))
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;;
   self)
  )

(defclass dsr_msgs::ParallelAxis2Response
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::ParallelAxis2Response
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

(defclass dsr_msgs::ParallelAxis2
  :super ros::object
  :slots ())

(setf (get dsr_msgs::ParallelAxis2 :md5sum-) "270e0e3f7b07b24362789c76324d4ac4")
(setf (get dsr_msgs::ParallelAxis2 :datatype-) "dsr_msgs/ParallelAxis2")
(setf (get dsr_msgs::ParallelAxis2 :request) dsr_msgs::ParallelAxis2Request)
(setf (get dsr_msgs::ParallelAxis2 :response) dsr_msgs::ParallelAxis2Response)

(defmethod dsr_msgs::ParallelAxis2Request
  (:response () (instance dsr_msgs::ParallelAxis2Response :init)))

(setf (get dsr_msgs::ParallelAxis2Request :md5sum-) "270e0e3f7b07b24362789c76324d4ac4")
(setf (get dsr_msgs::ParallelAxis2Request :datatype-) "dsr_msgs/ParallelAxis2Request")
(setf (get dsr_msgs::ParallelAxis2Request :definition-)
      "#____________________________________________________________________________________________
# parallel_axis(vect, axis, ref)  
#____________________________________________________________________________________________

float64[3] vect              # vector[3]  
int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
---
bool       success 
")

(setf (get dsr_msgs::ParallelAxis2Response :md5sum-) "270e0e3f7b07b24362789c76324d4ac4")
(setf (get dsr_msgs::ParallelAxis2Response :datatype-) "dsr_msgs/ParallelAxis2Response")
(setf (get dsr_msgs::ParallelAxis2Response :definition-)
      "#____________________________________________________________________________________________
# parallel_axis(vect, axis, ref)  
#____________________________________________________________________________________________

float64[3] vect              # vector[3]  
int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
---
bool       success 
")



(provide :dsr_msgs/ParallelAxis2 "270e0e3f7b07b24362789c76324d4ac4")


