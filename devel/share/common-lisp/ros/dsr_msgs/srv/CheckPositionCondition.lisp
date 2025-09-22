; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude CheckPositionCondition-request.msg.html

(cl:defclass <CheckPositionCondition-request> (roslisp-msg-protocol:ros-message)
  ((axis
    :reader axis
    :initarg :axis
    :type cl:fixnum
    :initform 0)
   (min
    :reader min
    :initarg :min
    :type cl:float
    :initform 0.0)
   (max
    :reader max
    :initarg :max
    :type cl:float
    :initform 0.0)
   (ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (pos
    :reader pos
    :initarg :pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass CheckPositionCondition-request (<CheckPositionCondition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckPositionCondition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckPositionCondition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<CheckPositionCondition-request> is deprecated: use dsr_msgs-srv:CheckPositionCondition-request instead.")))

(cl:ensure-generic-function 'axis-val :lambda-list '(m))
(cl:defmethod axis-val ((m <CheckPositionCondition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:axis-val is deprecated.  Use dsr_msgs-srv:axis instead.")
  (axis m))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <CheckPositionCondition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:min-val is deprecated.  Use dsr_msgs-srv:min instead.")
  (min m))

(cl:ensure-generic-function 'max-val :lambda-list '(m))
(cl:defmethod max-val ((m <CheckPositionCondition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:max-val is deprecated.  Use dsr_msgs-srv:max instead.")
  (max m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <CheckPositionCondition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <CheckPositionCondition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:mode-val is deprecated.  Use dsr_msgs-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <CheckPositionCondition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:pos-val is deprecated.  Use dsr_msgs-srv:pos instead.")
  (pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckPositionCondition-request>) ostream)
  "Serializes a message object of type '<CheckPositionCondition-request>"
  (cl:let* ((signed (cl:slot-value msg 'axis)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'pos))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckPositionCondition-request>) istream)
  "Deserializes a message object of type '<CheckPositionCondition-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'axis) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:setf (cl:slot-value msg 'pos) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'pos)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckPositionCondition-request>)))
  "Returns string type for a service object of type '<CheckPositionCondition-request>"
  "dsr_msgs/CheckPositionConditionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPositionCondition-request)))
  "Returns string type for a service object of type 'CheckPositionCondition-request"
  "dsr_msgs/CheckPositionConditionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckPositionCondition-request>)))
  "Returns md5sum for a message object of type '<CheckPositionCondition-request>"
  "d57ced9cfdce36e99486179c4a55944c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckPositionCondition-request)))
  "Returns md5sum for a message object of type 'CheckPositionCondition-request"
  "d57ced9cfdce36e99486179c4a55944c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckPositionCondition-request>)))
  "Returns full string definition for message of type '<CheckPositionCondition-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# check_position_condition  ~%#____________________________________________________________________________________________~%~%int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) ~%float64    min               # min    ~%float64    max               # max  ~%int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user_coordinate(101~~200)~%                             # <DR_WORLD is only available in M2.40 or later> ~%int8       mode #= 0         # DR_MV_MOD_ABS(0), DR_MV_MOD_REL(1) ~%float64[6] pos               # task pos(posx)  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckPositionCondition-request)))
  "Returns full string definition for message of type 'CheckPositionCondition-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# check_position_condition  ~%#____________________________________________________________________________________________~%~%int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) ~%float64    min               # min    ~%float64    max               # max  ~%int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user_coordinate(101~~200)~%                             # <DR_WORLD is only available in M2.40 or later> ~%int8       mode #= 0         # DR_MV_MOD_ABS(0), DR_MV_MOD_REL(1) ~%float64[6] pos               # task pos(posx)  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckPositionCondition-request>))
  (cl:+ 0
     1
     8
     8
     1
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckPositionCondition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckPositionCondition-request
    (cl:cons ':axis (axis msg))
    (cl:cons ':min (min msg))
    (cl:cons ':max (max msg))
    (cl:cons ':ref (ref msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':pos (pos msg))
))
;//! \htmlinclude CheckPositionCondition-response.msg.html

(cl:defclass <CheckPositionCondition-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CheckPositionCondition-response (<CheckPositionCondition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckPositionCondition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckPositionCondition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<CheckPositionCondition-response> is deprecated: use dsr_msgs-srv:CheckPositionCondition-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <CheckPositionCondition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckPositionCondition-response>) ostream)
  "Serializes a message object of type '<CheckPositionCondition-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckPositionCondition-response>) istream)
  "Deserializes a message object of type '<CheckPositionCondition-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckPositionCondition-response>)))
  "Returns string type for a service object of type '<CheckPositionCondition-response>"
  "dsr_msgs/CheckPositionConditionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPositionCondition-response)))
  "Returns string type for a service object of type 'CheckPositionCondition-response"
  "dsr_msgs/CheckPositionConditionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckPositionCondition-response>)))
  "Returns md5sum for a message object of type '<CheckPositionCondition-response>"
  "d57ced9cfdce36e99486179c4a55944c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckPositionCondition-response)))
  "Returns md5sum for a message object of type 'CheckPositionCondition-response"
  "d57ced9cfdce36e99486179c4a55944c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckPositionCondition-response>)))
  "Returns full string definition for message of type '<CheckPositionCondition-response>"
  (cl:format cl:nil "bool success                 # True or False~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckPositionCondition-response)))
  "Returns full string definition for message of type 'CheckPositionCondition-response"
  (cl:format cl:nil "bool success                 # True or False~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckPositionCondition-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckPositionCondition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckPositionCondition-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CheckPositionCondition)))
  'CheckPositionCondition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CheckPositionCondition)))
  'CheckPositionCondition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPositionCondition)))
  "Returns string type for a service object of type '<CheckPositionCondition>"
  "dsr_msgs/CheckPositionCondition")