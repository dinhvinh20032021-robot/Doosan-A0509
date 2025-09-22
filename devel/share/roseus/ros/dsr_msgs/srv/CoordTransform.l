;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::CoordTransform)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'CoordTransform (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::COORDTRANSFORM")
  (make-package "DSR_MSGS::COORDTRANSFORM"))
(unless (find-package "DSR_MSGS::COORDTRANSFORMREQUEST")
  (make-package "DSR_MSGS::COORDTRANSFORMREQUEST"))
(unless (find-package "DSR_MSGS::COORDTRANSFORMRESPONSE")
  (make-package "DSR_MSGS::COORDTRANSFORMRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::CoordTransformRequest
  :super ros::object
  :slots (_pos_in _ref_in _ref_out ))

(defmethod dsr_msgs::CoordTransformRequest
  (:init
   (&key
    ((:pos_in __pos_in) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:ref_in __ref_in) 0)
    ((:ref_out __ref_out) 0)
    )
   (send-super :init)
   (setq _pos_in __pos_in)
   (setq _ref_in (round __ref_in))
   (setq _ref_out (round __ref_out))
   self)
  (:pos_in
   (&optional __pos_in)
   (if __pos_in (setq _pos_in __pos_in)) _pos_in)
  (:ref_in
   (&optional __ref_in)
   (if __ref_in (setq _ref_in __ref_in)) _ref_in)
  (:ref_out
   (&optional __ref_out)
   (if __ref_out (setq _ref_out __ref_out)) _ref_out)
  (:serialization-length
   ()
   (+
    ;; float64[6] _pos_in
    (* 8    6)
    ;; int8 _ref_in
    1
    ;; int8 _ref_out
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _pos_in
     (dotimes (i 6)
       (sys::poke (elt _pos_in i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8 _ref_in
       (write-byte _ref_in s)
     ;; int8 _ref_out
       (write-byte _ref_out s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _pos_in
   (dotimes (i (length _pos_in))
     (setf (elt _pos_in i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int8 _ref_in
     (setq _ref_in (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref_in 127) (setq _ref_in (- _ref_in 256)))
   ;; int8 _ref_out
     (setq _ref_out (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref_out 127) (setq _ref_out (- _ref_out 256)))
   ;;
   self)
  )

(defclass dsr_msgs::CoordTransformResponse
  :super ros::object
  :slots (_conv_posx _success ))

(defmethod dsr_msgs::CoordTransformResponse
  (:init
   (&key
    ((:conv_posx __conv_posx) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _conv_posx __conv_posx)
   (setq _success __success)
   self)
  (:conv_posx
   (&optional __conv_posx)
   (if __conv_posx (setq _conv_posx __conv_posx)) _conv_posx)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float64[6] _conv_posx
    (* 8    6)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _conv_posx
     (dotimes (i 6)
       (sys::poke (elt _conv_posx i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _conv_posx
   (dotimes (i (length _conv_posx))
     (setf (elt _conv_posx i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::CoordTransform
  :super ros::object
  :slots ())

(setf (get dsr_msgs::CoordTransform :md5sum-) "0cb6558d600295c3a86e9d114e608d8a")
(setf (get dsr_msgs::CoordTransform :datatype-) "dsr_msgs/CoordTransform")
(setf (get dsr_msgs::CoordTransform :request) dsr_msgs::CoordTransformRequest)
(setf (get dsr_msgs::CoordTransform :response) dsr_msgs::CoordTransformResponse)

(defmethod dsr_msgs::CoordTransformRequest
  (:response () (instance dsr_msgs::CoordTransformResponse :init)))

(setf (get dsr_msgs::CoordTransformRequest :md5sum-) "0cb6558d600295c3a86e9d114e608d8a")
(setf (get dsr_msgs::CoordTransformRequest :datatype-) "dsr_msgs/CoordTransformRequest")
(setf (get dsr_msgs::CoordTransformRequest :definition-)
      "#____________________________________________________________________________________________
# coord_transform   
#____________________________________________________________________________________________

float64[6] pos_in            # task pos(posx)  
int8       ref_in            # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
int8       ref_out           # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200) 
                             # <ref is only available in M2.40 or later> 
---
float64[6] conv_posx         # task pos(posx)
bool       success
")

(setf (get dsr_msgs::CoordTransformResponse :md5sum-) "0cb6558d600295c3a86e9d114e608d8a")
(setf (get dsr_msgs::CoordTransformResponse :datatype-) "dsr_msgs/CoordTransformResponse")
(setf (get dsr_msgs::CoordTransformResponse :definition-)
      "#____________________________________________________________________________________________
# coord_transform   
#____________________________________________________________________________________________

float64[6] pos_in            # task pos(posx)  
int8       ref_in            # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
int8       ref_out           # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200) 
                             # <ref is only available in M2.40 or later> 
---
float64[6] conv_posx         # task pos(posx)
bool       success
")



(provide :dsr_msgs/CoordTransform "0cb6558d600295c3a86e9d114e608d8a")


