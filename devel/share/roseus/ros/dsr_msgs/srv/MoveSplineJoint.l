;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::MoveSplineJoint)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'MoveSplineJoint (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::MOVESPLINEJOINT")
  (make-package "DSR_MSGS::MOVESPLINEJOINT"))
(unless (find-package "DSR_MSGS::MOVESPLINEJOINTREQUEST")
  (make-package "DSR_MSGS::MOVESPLINEJOINTREQUEST"))
(unless (find-package "DSR_MSGS::MOVESPLINEJOINTRESPONSE")
  (make-package "DSR_MSGS::MOVESPLINEJOINTRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass dsr_msgs::MoveSplineJointRequest
  :super ros::object
  :slots (_pos _posCnt _vel _acc _time _mode _syncType ))

(defmethod dsr_msgs::MoveSplineJointRequest
  (:init
   (&key
    ((:pos __pos) ())
    ((:posCnt __posCnt) 0)
    ((:vel __vel) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:acc __acc) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:time __time) 0.0)
    ((:mode __mode) 0)
    ((:syncType __syncType) 0)
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _posCnt (round __posCnt))
   (setq _vel __vel)
   (setq _acc __acc)
   (setq _time (float __time))
   (setq _mode (round __mode))
   (setq _syncType (round __syncType))
   self)
  (:pos
   (&rest __pos)
   (if (keywordp (car __pos))
       (send* _pos __pos)
     (progn
       (if __pos (setq _pos (car __pos)))
       _pos)))
  (:posCnt
   (&optional __posCnt)
   (if __posCnt (setq _posCnt __posCnt)) _posCnt)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:syncType
   (&optional __syncType)
   (if __syncType (setq _syncType __syncType)) _syncType)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64MultiArray[] _pos
    (apply #'+ (send-all _pos :serialization-length)) 4
    ;; int8 _posCnt
    1
    ;; float64[6] _vel
    (* 8    6)
    ;; float64[6] _acc
    (* 8    6)
    ;; float64 _time
    8
    ;; int8 _mode
    1
    ;; int8 _syncType
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64MultiArray[] _pos
     (write-long (length _pos) s)
     (dolist (elem _pos)
       (send elem :serialize s)
       )
     ;; int8 _posCnt
       (write-byte _posCnt s)
     ;; float64[6] _vel
     (dotimes (i 6)
       (sys::poke (elt _vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _acc
     (dotimes (i 6)
       (sys::poke (elt _acc i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8 _mode
       (write-byte _mode s)
     ;; int8 _syncType
       (write-byte _syncType s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64MultiArray[] _pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pos (let (r) (dotimes (i n) (push (instance std_msgs::Float64MultiArray :init) r)) r))
     (dolist (elem- _pos)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int8 _posCnt
     (setq _posCnt (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _posCnt 127) (setq _posCnt (- _posCnt 256)))
   ;; float64[6] _vel
   (dotimes (i (length _vel))
     (setf (elt _vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _acc
   (dotimes (i (length _acc))
     (setf (elt _acc i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;; int8 _syncType
     (setq _syncType (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _syncType 127) (setq _syncType (- _syncType 256)))
   ;;
   self)
  )

(defclass dsr_msgs::MoveSplineJointResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::MoveSplineJointResponse
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

(defclass dsr_msgs::MoveSplineJoint
  :super ros::object
  :slots ())

(setf (get dsr_msgs::MoveSplineJoint :md5sum-) "38e3ecc2960089549a21015a74e932c2")
(setf (get dsr_msgs::MoveSplineJoint :datatype-) "dsr_msgs/MoveSplineJoint")
(setf (get dsr_msgs::MoveSplineJoint :request) dsr_msgs::MoveSplineJointRequest)
(setf (get dsr_msgs::MoveSplineJoint :response) dsr_msgs::MoveSplineJointResponse)

(defmethod dsr_msgs::MoveSplineJointRequest
  (:response () (instance dsr_msgs::MoveSplineJointResponse :init)))

(setf (get dsr_msgs::MoveSplineJointRequest :md5sum-) "38e3ecc2960089549a21015a74e932c2")
(setf (get dsr_msgs::MoveSplineJointRequest :datatype-) "dsr_msgs/MoveSplineJointRequest")
(setf (get dsr_msgs::MoveSplineJointRequest :definition-)
      "#____________________________________________________________________________________________
# move_spline_joint  
###float64[100][6] pos         # target
#____________________________________________________________________________________________

std_msgs/Float64MultiArray[] pos         # target [100][6] pos
int8       posCnt                        # target cnt 
float64[6] vel                           # set joint velocity: [deg/sec]
float64[6] acc                           # set joint acceleration: [deg/sec2] 
float64    time #= 0.0                   # Time [sec] 
int8       mode #= 0                     # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
int8       syncType #=0                  # SYNC = 0, ASYNC = 1

================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
---
bool success
")

(setf (get dsr_msgs::MoveSplineJointResponse :md5sum-) "38e3ecc2960089549a21015a74e932c2")
(setf (get dsr_msgs::MoveSplineJointResponse :datatype-) "dsr_msgs/MoveSplineJointResponse")
(setf (get dsr_msgs::MoveSplineJointResponse :definition-)
      "#____________________________________________________________________________________________
# move_spline_joint  
###float64[100][6] pos         # target
#____________________________________________________________________________________________

std_msgs/Float64MultiArray[] pos         # target [100][6] pos
int8       posCnt                        # target cnt 
float64[6] vel                           # set joint velocity: [deg/sec]
float64[6] acc                           # set joint acceleration: [deg/sec2] 
float64    time #= 0.0                   # Time [sec] 
int8       mode #= 0                     # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
int8       syncType #=0                  # SYNC = 0, ASYNC = 1

================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
---
bool success
")



(provide :dsr_msgs/MoveSplineJoint "38e3ecc2960089549a21015a74e932c2")


