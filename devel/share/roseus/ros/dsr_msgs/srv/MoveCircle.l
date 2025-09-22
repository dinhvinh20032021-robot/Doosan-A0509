;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::MoveCircle)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'MoveCircle (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::MOVECIRCLE")
  (make-package "DSR_MSGS::MOVECIRCLE"))
(unless (find-package "DSR_MSGS::MOVECIRCLEREQUEST")
  (make-package "DSR_MSGS::MOVECIRCLEREQUEST"))
(unless (find-package "DSR_MSGS::MOVECIRCLERESPONSE")
  (make-package "DSR_MSGS::MOVECIRCLERESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass dsr_msgs::MoveCircleRequest
  :super ros::object
  :slots (_pos _vel _acc _time _radius _ref _mode _angle1 _angle2 _blendType _syncType ))

(defmethod dsr_msgs::MoveCircleRequest
  (:init
   (&key
    ((:pos __pos) ())
    ((:vel __vel) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:acc __acc) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:time __time) 0.0)
    ((:radius __radius) 0.0)
    ((:ref __ref) 0)
    ((:mode __mode) 0)
    ((:angle1 __angle1) 0.0)
    ((:angle2 __angle2) 0.0)
    ((:blendType __blendType) 0)
    ((:syncType __syncType) 0)
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _vel __vel)
   (setq _acc __acc)
   (setq _time (float __time))
   (setq _radius (float __radius))
   (setq _ref (round __ref))
   (setq _mode (round __mode))
   (setq _angle1 (float __angle1))
   (setq _angle2 (float __angle2))
   (setq _blendType (round __blendType))
   (setq _syncType (round __syncType))
   self)
  (:pos
   (&rest __pos)
   (if (keywordp (car __pos))
       (send* _pos __pos)
     (progn
       (if __pos (setq _pos (car __pos)))
       _pos)))
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:angle1
   (&optional __angle1)
   (if __angle1 (setq _angle1 __angle1)) _angle1)
  (:angle2
   (&optional __angle2)
   (if __angle2 (setq _angle2 __angle2)) _angle2)
  (:blendType
   (&optional __blendType)
   (if __blendType (setq _blendType __blendType)) _blendType)
  (:syncType
   (&optional __syncType)
   (if __syncType (setq _syncType __syncType)) _syncType)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64MultiArray[] _pos
    (apply #'+ (send-all _pos :serialization-length)) 4
    ;; float64[2] _vel
    (* 8    2)
    ;; float64[2] _acc
    (* 8    2)
    ;; float64 _time
    8
    ;; float64 _radius
    8
    ;; int8 _ref
    1
    ;; int8 _mode
    1
    ;; float64 _angle1
    8
    ;; float64 _angle2
    8
    ;; int8 _blendType
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
     ;; float64[2] _vel
     (dotimes (i 2)
       (sys::poke (elt _vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[2] _acc
     (dotimes (i 2)
       (sys::poke (elt _acc i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8 _ref
       (write-byte _ref s)
     ;; int8 _mode
       (write-byte _mode s)
     ;; float64 _angle1
       (sys::poke _angle1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angle2
       (sys::poke _angle2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8 _blendType
       (write-byte _blendType s)
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
   ;; float64[2] _vel
   (dotimes (i (length _vel))
     (setf (elt _vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[2] _acc
   (dotimes (i (length _acc))
     (setf (elt _acc i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _radius
     (setq _radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;; int8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;; float64 _angle1
     (setq _angle1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angle2
     (setq _angle2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8 _blendType
     (setq _blendType (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _blendType 127) (setq _blendType (- _blendType 256)))
   ;; int8 _syncType
     (setq _syncType (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _syncType 127) (setq _syncType (- _syncType 256)))
   ;;
   self)
  )

(defclass dsr_msgs::MoveCircleResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::MoveCircleResponse
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

(defclass dsr_msgs::MoveCircle
  :super ros::object
  :slots ())

(setf (get dsr_msgs::MoveCircle :md5sum-) "54ef8d9932dcc86f5742e22ae7290ca5")
(setf (get dsr_msgs::MoveCircle :datatype-) "dsr_msgs/MoveCircle")
(setf (get dsr_msgs::MoveCircle :request) dsr_msgs::MoveCircleRequest)
(setf (get dsr_msgs::MoveCircle :response) dsr_msgs::MoveCircleResponse)

(defmethod dsr_msgs::MoveCircleRequest
  (:response () (instance dsr_msgs::MoveCircleResponse :init)))

(setf (get dsr_msgs::MoveCircleRequest :md5sum-) "54ef8d9932dcc86f5742e22ae7290ca5")
(setf (get dsr_msgs::MoveCircleRequest :datatype-) "dsr_msgs/MoveCircleRequest")
(setf (get dsr_msgs::MoveCircleRequest :definition-)
      "#____________________________________________________________________________________________
# move_circle  
#____________________________________________________________________________________________

std_msgs/Float64MultiArray[] pos  # target[2][6]  
float64[2]      vel               # set velocity: [mm/sec], [deg/sec]
float64[2]      acc               # set acceleration: [mm/sec2], [deg/sec2]
float64         time #= 0.0       # Time [sec] 
float64         radius #=0.0      # Radius under blending mode [mm] 
int8            ref               # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                                  # <DR_WORLD is only available in M2.40 or later> 
int8            mode #= 0         # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
float64         angle1 #= 0.0     # angle1 [degree]
float64         angle2 #= 0.0     # angle2 [degree]
int8            blendType #= 0    # BLENDING_SPEED_TYPE_DUPLICATE=0, BLENDING_SPEED_TYPE_OVERRIDE=1
int8            syncType #=0      # SYNC = 0, ASYNC = 1

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

(setf (get dsr_msgs::MoveCircleResponse :md5sum-) "54ef8d9932dcc86f5742e22ae7290ca5")
(setf (get dsr_msgs::MoveCircleResponse :datatype-) "dsr_msgs/MoveCircleResponse")
(setf (get dsr_msgs::MoveCircleResponse :definition-)
      "#____________________________________________________________________________________________
# move_circle  
#____________________________________________________________________________________________

std_msgs/Float64MultiArray[] pos  # target[2][6]  
float64[2]      vel               # set velocity: [mm/sec], [deg/sec]
float64[2]      acc               # set acceleration: [mm/sec2], [deg/sec2]
float64         time #= 0.0       # Time [sec] 
float64         radius #=0.0      # Radius under blending mode [mm] 
int8            ref               # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                                  # <DR_WORLD is only available in M2.40 or later> 
int8            mode #= 0         # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
float64         angle1 #= 0.0     # angle1 [degree]
float64         angle2 #= 0.0     # angle2 [degree]
int8            blendType #= 0    # BLENDING_SPEED_TYPE_DUPLICATE=0, BLENDING_SPEED_TYPE_OVERRIDE=1
int8            syncType #=0      # SYNC = 0, ASYNC = 1

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



(provide :dsr_msgs/MoveCircle "54ef8d9932dcc86f5742e22ae7290ca5")


