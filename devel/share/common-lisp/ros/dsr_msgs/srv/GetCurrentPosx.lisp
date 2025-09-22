; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetCurrentPosx-request.msg.html

(cl:defclass <GetCurrentPosx-request> (roslisp-msg-protocol:ros-message)
  ((ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetCurrentPosx-request (<GetCurrentPosx-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentPosx-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentPosx-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCurrentPosx-request> is deprecated: use dsr_msgs-srv:GetCurrentPosx-request instead.")))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <GetCurrentPosx-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentPosx-request>) ostream)
  "Serializes a message object of type '<GetCurrentPosx-request>"
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentPosx-request>) istream)
  "Deserializes a message object of type '<GetCurrentPosx-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentPosx-request>)))
  "Returns string type for a service object of type '<GetCurrentPosx-request>"
  "dsr_msgs/GetCurrentPosxRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentPosx-request)))
  "Returns string type for a service object of type 'GetCurrentPosx-request"
  "dsr_msgs/GetCurrentPosxRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentPosx-request>)))
  "Returns md5sum for a message object of type '<GetCurrentPosx-request>"
  "a35de69e7061270e867fdb5acbc17c91")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentPosx-request)))
  "Returns md5sum for a message object of type 'GetCurrentPosx-request"
  "a35de69e7061270e867fdb5acbc17c91")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentPosx-request>)))
  "Returns full string definition for message of type '<GetCurrentPosx-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_current_posx()  ~%#____________________________________________________________________________________________~%# This service returns the current task position.~%~%int8       ref               # DR_BASE(0), DR_WORLD(2), user coord(101~~200)~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentPosx-request)))
  "Returns full string definition for message of type 'GetCurrentPosx-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_current_posx()  ~%#____________________________________________________________________________________________~%# This service returns the current task position.~%~%int8       ref               # DR_BASE(0), DR_WORLD(2), user coord(101~~200)~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentPosx-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentPosx-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentPosx-request
    (cl:cons ':ref (ref msg))
))
;//! \htmlinclude GetCurrentPosx-response.msg.html

(cl:defclass <GetCurrentPosx-response> (roslisp-msg-protocol:ros-message)
  ((task_pos_info
    :reader task_pos_info
    :initarg :task_pos_info
    :type (cl:vector std_msgs-msg:Float64MultiArray)
   :initform (cl:make-array 0 :element-type 'std_msgs-msg:Float64MultiArray :initial-element (cl:make-instance 'std_msgs-msg:Float64MultiArray)))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetCurrentPosx-response (<GetCurrentPosx-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentPosx-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentPosx-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCurrentPosx-response> is deprecated: use dsr_msgs-srv:GetCurrentPosx-response instead.")))

(cl:ensure-generic-function 'task_pos_info-val :lambda-list '(m))
(cl:defmethod task_pos_info-val ((m <GetCurrentPosx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:task_pos_info-val is deprecated.  Use dsr_msgs-srv:task_pos_info instead.")
  (task_pos_info m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetCurrentPosx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentPosx-response>) ostream)
  "Serializes a message object of type '<GetCurrentPosx-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'task_pos_info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'task_pos_info))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentPosx-response>) istream)
  "Deserializes a message object of type '<GetCurrentPosx-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'task_pos_info) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'task_pos_info)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'std_msgs-msg:Float64MultiArray))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentPosx-response>)))
  "Returns string type for a service object of type '<GetCurrentPosx-response>"
  "dsr_msgs/GetCurrentPosxResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentPosx-response)))
  "Returns string type for a service object of type 'GetCurrentPosx-response"
  "dsr_msgs/GetCurrentPosxResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentPosx-response>)))
  "Returns md5sum for a message object of type '<GetCurrentPosx-response>"
  "a35de69e7061270e867fdb5acbc17c91")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentPosx-response)))
  "Returns md5sum for a message object of type 'GetCurrentPosx-response"
  "a35de69e7061270e867fdb5acbc17c91")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentPosx-response>)))
  "Returns full string definition for message of type '<GetCurrentPosx-response>"
  (cl:format cl:nil "std_msgs/Float64MultiArray[] task_pos_info  # task pos = task_pos_info[0][0:5], solution sapce = task_pos_info[0][6]~%bool        success   ~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentPosx-response)))
  "Returns full string definition for message of type 'GetCurrentPosx-response"
  (cl:format cl:nil "std_msgs/Float64MultiArray[] task_pos_info  # task pos = task_pos_info[0][0:5], solution sapce = task_pos_info[0][6]~%bool        success   ~%~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentPosx-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'task_pos_info) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentPosx-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentPosx-response
    (cl:cons ':task_pos_info (task_pos_info msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCurrentPosx)))
  'GetCurrentPosx-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCurrentPosx)))
  'GetCurrentPosx-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentPosx)))
  "Returns string type for a service object of type '<GetCurrentPosx>"
  "dsr_msgs/GetCurrentPosx")