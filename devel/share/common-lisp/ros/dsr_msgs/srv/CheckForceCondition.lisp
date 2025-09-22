; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude CheckForceCondition-request.msg.html

(cl:defclass <CheckForceCondition-request> (roslisp-msg-protocol:ros-message)
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
    :initform 0))
)

(cl:defclass CheckForceCondition-request (<CheckForceCondition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckForceCondition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckForceCondition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<CheckForceCondition-request> is deprecated: use dsr_msgs-srv:CheckForceCondition-request instead.")))

(cl:ensure-generic-function 'axis-val :lambda-list '(m))
(cl:defmethod axis-val ((m <CheckForceCondition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:axis-val is deprecated.  Use dsr_msgs-srv:axis instead.")
  (axis m))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <CheckForceCondition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:min-val is deprecated.  Use dsr_msgs-srv:min instead.")
  (min m))

(cl:ensure-generic-function 'max-val :lambda-list '(m))
(cl:defmethod max-val ((m <CheckForceCondition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:max-val is deprecated.  Use dsr_msgs-srv:max instead.")
  (max m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <CheckForceCondition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckForceCondition-request>) ostream)
  "Serializes a message object of type '<CheckForceCondition-request>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckForceCondition-request>) istream)
  "Deserializes a message object of type '<CheckForceCondition-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckForceCondition-request>)))
  "Returns string type for a service object of type '<CheckForceCondition-request>"
  "dsr_msgs/CheckForceConditionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckForceCondition-request)))
  "Returns string type for a service object of type 'CheckForceCondition-request"
  "dsr_msgs/CheckForceConditionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckForceCondition-request>)))
  "Returns md5sum for a message object of type '<CheckForceCondition-request>"
  "4904883ec6cc91634eacedbab3722558")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckForceCondition-request)))
  "Returns md5sum for a message object of type 'CheckForceCondition-request"
  "4904883ec6cc91634eacedbab3722558")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckForceCondition-request>)))
  "Returns full string definition for message of type '<CheckForceCondition-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# check_force_condition ~%#This service checks the status of the given force. It disregards the force direction and only compares the sizes. ~%#This condition can be repeated with the while or if statement. Measuring the force, axis is based on the ref coordinate and measuring the moment,~%#axis is based on the tool coordinate.~%#____________________________________________________________________________________________~%~%int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2), DR_AXIS_A(10), DR_AXIS_B(11), DR_AXIS_C(12) ~%float64    min               # min >=0.0   ~%float64    max               # max >=0.0 ~%int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~~200)~%                             # <DR_WORLD is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckForceCondition-request)))
  "Returns full string definition for message of type 'CheckForceCondition-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# check_force_condition ~%#This service checks the status of the given force. It disregards the force direction and only compares the sizes. ~%#This condition can be repeated with the while or if statement. Measuring the force, axis is based on the ref coordinate and measuring the moment,~%#axis is based on the tool coordinate.~%#____________________________________________________________________________________________~%~%int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2), DR_AXIS_A(10), DR_AXIS_B(11), DR_AXIS_C(12) ~%float64    min               # min >=0.0   ~%float64    max               # max >=0.0 ~%int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~~200)~%                             # <DR_WORLD is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckForceCondition-request>))
  (cl:+ 0
     1
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckForceCondition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckForceCondition-request
    (cl:cons ':axis (axis msg))
    (cl:cons ':min (min msg))
    (cl:cons ':max (max msg))
    (cl:cons ':ref (ref msg))
))
;//! \htmlinclude CheckForceCondition-response.msg.html

(cl:defclass <CheckForceCondition-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CheckForceCondition-response (<CheckForceCondition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckForceCondition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckForceCondition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<CheckForceCondition-response> is deprecated: use dsr_msgs-srv:CheckForceCondition-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <CheckForceCondition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckForceCondition-response>) ostream)
  "Serializes a message object of type '<CheckForceCondition-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckForceCondition-response>) istream)
  "Deserializes a message object of type '<CheckForceCondition-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckForceCondition-response>)))
  "Returns string type for a service object of type '<CheckForceCondition-response>"
  "dsr_msgs/CheckForceConditionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckForceCondition-response)))
  "Returns string type for a service object of type 'CheckForceCondition-response"
  "dsr_msgs/CheckForceConditionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckForceCondition-response>)))
  "Returns md5sum for a message object of type '<CheckForceCondition-response>"
  "4904883ec6cc91634eacedbab3722558")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckForceCondition-response)))
  "Returns md5sum for a message object of type 'CheckForceCondition-response"
  "4904883ec6cc91634eacedbab3722558")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckForceCondition-response>)))
  "Returns full string definition for message of type '<CheckForceCondition-response>"
  (cl:format cl:nil "bool       success                 # True or False~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckForceCondition-response)))
  "Returns full string definition for message of type 'CheckForceCondition-response"
  (cl:format cl:nil "bool       success                 # True or False~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckForceCondition-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckForceCondition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckForceCondition-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CheckForceCondition)))
  'CheckForceCondition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CheckForceCondition)))
  'CheckForceCondition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckForceCondition)))
  "Returns string type for a service object of type '<CheckForceCondition>"
  "dsr_msgs/CheckForceCondition")