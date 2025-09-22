;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetCurrentPosx)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetCurrentPosx (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETCURRENTPOSX")
  (make-package "DSR_MSGS::GETCURRENTPOSX"))
(unless (find-package "DSR_MSGS::GETCURRENTPOSXREQUEST")
  (make-package "DSR_MSGS::GETCURRENTPOSXREQUEST"))
(unless (find-package "DSR_MSGS::GETCURRENTPOSXRESPONSE")
  (make-package "DSR_MSGS::GETCURRENTPOSXRESPONSE"))

(in-package "ROS")



(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dsr_msgs::GetCurrentPosxRequest
  :super ros::object
  :slots (_ref ))

(defmethod dsr_msgs::GetCurrentPosxRequest
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

(defclass dsr_msgs::GetCurrentPosxResponse
  :super ros::object
  :slots (_task_pos_info _success ))

(defmethod dsr_msgs::GetCurrentPosxResponse
  (:init
   (&key
    ((:task_pos_info __task_pos_info) ())
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _task_pos_info __task_pos_info)
   (setq _success __success)
   self)
  (:task_pos_info
   (&rest __task_pos_info)
   (if (keywordp (car __task_pos_info))
       (send* _task_pos_info __task_pos_info)
     (progn
       (if __task_pos_info (setq _task_pos_info (car __task_pos_info)))
       _task_pos_info)))
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64MultiArray[] _task_pos_info
    (apply #'+ (send-all _task_pos_info :serialization-length)) 4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64MultiArray[] _task_pos_info
     (write-long (length _task_pos_info) s)
     (dolist (elem _task_pos_info)
       (send elem :serialize s)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64MultiArray[] _task_pos_info
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _task_pos_info (let (r) (dotimes (i n) (push (instance std_msgs::Float64MultiArray :init) r)) r))
     (dolist (elem- _task_pos_info)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetCurrentPosx
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetCurrentPosx :md5sum-) "a35de69e7061270e867fdb5acbc17c91")
(setf (get dsr_msgs::GetCurrentPosx :datatype-) "dsr_msgs/GetCurrentPosx")
(setf (get dsr_msgs::GetCurrentPosx :request) dsr_msgs::GetCurrentPosxRequest)
(setf (get dsr_msgs::GetCurrentPosx :response) dsr_msgs::GetCurrentPosxResponse)

(defmethod dsr_msgs::GetCurrentPosxRequest
  (:response () (instance dsr_msgs::GetCurrentPosxResponse :init)))

(setf (get dsr_msgs::GetCurrentPosxRequest :md5sum-) "a35de69e7061270e867fdb5acbc17c91")
(setf (get dsr_msgs::GetCurrentPosxRequest :datatype-) "dsr_msgs/GetCurrentPosxRequest")
(setf (get dsr_msgs::GetCurrentPosxRequest :definition-)
      "#____________________________________________________________________________________________
# get_current_posx()  
#____________________________________________________________________________________________
# This service returns the current task position.

int8       ref               # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
---
std_msgs/Float64MultiArray[] task_pos_info  # task pos = task_pos_info[0][0:5], solution sapce = task_pos_info[0][6]
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

(setf (get dsr_msgs::GetCurrentPosxResponse :md5sum-) "a35de69e7061270e867fdb5acbc17c91")
(setf (get dsr_msgs::GetCurrentPosxResponse :datatype-) "dsr_msgs/GetCurrentPosxResponse")
(setf (get dsr_msgs::GetCurrentPosxResponse :definition-)
      "#____________________________________________________________________________________________
# get_current_posx()  
#____________________________________________________________________________________________
# This service returns the current task position.

int8       ref               # DR_BASE(0), DR_WORLD(2), user coord(101~200)
                             # <ref is only available in M2.40 or later> 
---
std_msgs/Float64MultiArray[] task_pos_info  # task pos = task_pos_info[0][0:5], solution sapce = task_pos_info[0][6]
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



(provide :dsr_msgs/GetCurrentPosx "a35de69e7061270e867fdb5acbc17c91")


