;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::AlignAxis1)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'AlignAxis1 (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::ALIGNAXIS1")
  (make-package "DSR_MSGS::ALIGNAXIS1"))
(unless (find-package "DSR_MSGS::ALIGNAXIS1REQUEST")
  (make-package "DSR_MSGS::ALIGNAXIS1REQUEST"))
(unless (find-package "DSR_MSGS::ALIGNAXIS1RESPONSE")
  (make-package "DSR_MSGS::ALIGNAXIS1RESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::AlignAxis1Request
  :super ros::object
  :slots (_x1 _x2 _x3 _source_vect _axis _ref ))

(defmethod dsr_msgs::AlignAxis1Request
  (:init
   (&key
    ((:x1 __x1) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:x2 __x2) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:x3 __x3) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:source_vect __source_vect) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:axis __axis) 0)
    ((:ref __ref) 0)
    )
   (send-super :init)
   (setq _x1 __x1)
   (setq _x2 __x2)
   (setq _x3 __x3)
   (setq _source_vect __source_vect)
   (setq _axis (round __axis))
   (setq _ref (round __ref))
   self)
  (:x1
   (&optional __x1)
   (if __x1 (setq _x1 __x1)) _x1)
  (:x2
   (&optional __x2)
   (if __x2 (setq _x2 __x2)) _x2)
  (:x3
   (&optional __x3)
   (if __x3 (setq _x3 __x3)) _x3)
  (:source_vect
   (&optional __source_vect)
   (if __source_vect (setq _source_vect __source_vect)) _source_vect)
  (:axis
   (&optional __axis)
   (if __axis (setq _axis __axis)) _axis)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:serialization-length
   ()
   (+
    ;; float64[6] _x1
    (* 8    6)
    ;; float64[6] _x2
    (* 8    6)
    ;; float64[6] _x3
    (* 8    6)
    ;; float64[3] _source_vect
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
     ;; float64[6] _x1
     (dotimes (i 6)
       (sys::poke (elt _x1 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _x2
     (dotimes (i 6)
       (sys::poke (elt _x2 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _x3
     (dotimes (i 6)
       (sys::poke (elt _x3 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _source_vect
     (dotimes (i 3)
       (sys::poke (elt _source_vect i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8 _axis
       (write-byte _axis s)
     ;; int8 _ref
       (write-byte _ref s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _x1
   (dotimes (i (length _x1))
     (setf (elt _x1 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _x2
   (dotimes (i (length _x2))
     (setf (elt _x2 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _x3
   (dotimes (i (length _x3))
     (setf (elt _x3 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _source_vect
   (dotimes (i (length _source_vect))
     (setf (elt _source_vect i) (sys::peek buf ptr- :double)) (incf ptr- 8)
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

(defclass dsr_msgs::AlignAxis1Response
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::AlignAxis1Response
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

(defclass dsr_msgs::AlignAxis1
  :super ros::object
  :slots ())

(setf (get dsr_msgs::AlignAxis1 :md5sum-) "bc982fa1299790397f690022cfb7030c")
(setf (get dsr_msgs::AlignAxis1 :datatype-) "dsr_msgs/AlignAxis1")
(setf (get dsr_msgs::AlignAxis1 :request) dsr_msgs::AlignAxis1Request)
(setf (get dsr_msgs::AlignAxis1 :response) dsr_msgs::AlignAxis1Response)

(defmethod dsr_msgs::AlignAxis1Request
  (:response () (instance dsr_msgs::AlignAxis1Response :init)))

(setf (get dsr_msgs::AlignAxis1Request :md5sum-) "bc982fa1299790397f690022cfb7030c")
(setf (get dsr_msgs::AlignAxis1Request :datatype-) "dsr_msgs/AlignAxis1Request")
(setf (get dsr_msgs::AlignAxis1Request :definition-)
      "#____________________________________________________________________________________________
# align_axis(x1, x2, x3, source_vect, axis, ref)
#____________________________________________________________________________________________

float64[6] x1                # task pos(posx)  
float64[6] x2                # task pos(posx)  
float64[6] x3                # task pos(posx)
float64[3] source_vect       # source vector[3]  
int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
int8       ref               # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
---
bool       success
")

(setf (get dsr_msgs::AlignAxis1Response :md5sum-) "bc982fa1299790397f690022cfb7030c")
(setf (get dsr_msgs::AlignAxis1Response :datatype-) "dsr_msgs/AlignAxis1Response")
(setf (get dsr_msgs::AlignAxis1Response :definition-)
      "#____________________________________________________________________________________________
# align_axis(x1, x2, x3, source_vect, axis, ref)
#____________________________________________________________________________________________

float64[6] x1                # task pos(posx)  
float64[6] x2                # task pos(posx)  
float64[6] x3                # task pos(posx)
float64[3] source_vect       # source vector[3]  
int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
int8       ref               # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
---
bool       success
")



(provide :dsr_msgs/AlignAxis1 "bc982fa1299790397f690022cfb7030c")


