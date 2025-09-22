;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetCurrentRotm)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetCurrentRotm (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETCURRENTROTM")
  (make-package "DSR_MSGS::GETCURRENTROTM"))
(unless (find-package "DSR_MSGS::GETCURRENTROTMREQUEST")
  (make-package "DSR_MSGS::GETCURRENTROTMREQUEST"))
(unless (find-package "DSR_MSGS::GETCURRENTROTMRESPONSE")
  (make-package "DSR_MSGS::GETCURRENTROTMRESPONSE"))

(in-package "ROS")



(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dsr_msgs::GetCurrentRotmRequest
  :super ros::object
  :slots (_ref ))

(defmethod dsr_msgs::GetCurrentRotmRequest
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

(defclass dsr_msgs::GetCurrentRotmResponse
  :super ros::object
  :slots (_rot_matrix _success ))

(defmethod dsr_msgs::GetCurrentRotmResponse
  (:init
   (&key
    ((:rot_matrix __rot_matrix) ())
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _rot_matrix __rot_matrix)
   (setq _success __success)
   self)
  (:rot_matrix
   (&rest __rot_matrix)
   (if (keywordp (car __rot_matrix))
       (send* _rot_matrix __rot_matrix)
     (progn
       (if __rot_matrix (setq _rot_matrix (car __rot_matrix)))
       _rot_matrix)))
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64MultiArray[] _rot_matrix
    (apply #'+ (send-all _rot_matrix :serialization-length)) 4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64MultiArray[] _rot_matrix
     (write-long (length _rot_matrix) s)
     (dolist (elem _rot_matrix)
       (send elem :serialize s)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64MultiArray[] _rot_matrix
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rot_matrix (let (r) (dotimes (i n) (push (instance std_msgs::Float64MultiArray :init) r)) r))
     (dolist (elem- _rot_matrix)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetCurrentRotm
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetCurrentRotm :md5sum-) "b547f5d15d7bef34dd869e770e7e403f")
(setf (get dsr_msgs::GetCurrentRotm :datatype-) "dsr_msgs/GetCurrentRotm")
(setf (get dsr_msgs::GetCurrentRotm :request) dsr_msgs::GetCurrentRotmRequest)
(setf (get dsr_msgs::GetCurrentRotm :response) dsr_msgs::GetCurrentRotmResponse)

(defmethod dsr_msgs::GetCurrentRotmRequest
  (:response () (instance dsr_msgs::GetCurrentRotmResponse :init)))

(setf (get dsr_msgs::GetCurrentRotmRequest :md5sum-) "b547f5d15d7bef34dd869e770e7e403f")
(setf (get dsr_msgs::GetCurrentRotmRequest :datatype-) "dsr_msgs/GetCurrentRotmRequest")
(setf (get dsr_msgs::GetCurrentRotmRequest :definition-)
      "#____________________________________________________________________________________________
# get_current_rotm(ref)  
#____________________________________________________________________________________________
# This service returns the direction and matrix of the current tool based on the ref coordinate.

int8        ref               # DR_BASE(0), DR_WORLD(2)
---
std_msgs/Float64MultiArray[] rot_matrix  # target[3][3] Rotation matrix
bool        success

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
")

(setf (get dsr_msgs::GetCurrentRotmResponse :md5sum-) "b547f5d15d7bef34dd869e770e7e403f")
(setf (get dsr_msgs::GetCurrentRotmResponse :datatype-) "dsr_msgs/GetCurrentRotmResponse")
(setf (get dsr_msgs::GetCurrentRotmResponse :definition-)
      "#____________________________________________________________________________________________
# get_current_rotm(ref)  
#____________________________________________________________________________________________
# This service returns the direction and matrix of the current tool based on the ref coordinate.

int8        ref               # DR_BASE(0), DR_WORLD(2)
---
std_msgs/Float64MultiArray[] rot_matrix  # target[3][3] Rotation matrix
bool        success

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
")



(provide :dsr_msgs/GetCurrentRotm "b547f5d15d7bef34dd869e770e7e403f")


