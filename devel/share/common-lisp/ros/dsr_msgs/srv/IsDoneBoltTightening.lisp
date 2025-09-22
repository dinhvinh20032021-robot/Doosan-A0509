; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude IsDoneBoltTightening-request.msg.html

(cl:defclass <IsDoneBoltTightening-request> (roslisp-msg-protocol:ros-message)
  ((m
    :reader m
    :initarg :m
    :type cl:float
    :initform 0.0)
   (timeout
    :reader timeout
    :initarg :timeout
    :type cl:float
    :initform 0.0)
   (axis
    :reader axis
    :initarg :axis
    :type cl:fixnum
    :initform 0))
)

(cl:defclass IsDoneBoltTightening-request (<IsDoneBoltTightening-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IsDoneBoltTightening-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IsDoneBoltTightening-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<IsDoneBoltTightening-request> is deprecated: use dsr_msgs-srv:IsDoneBoltTightening-request instead.")))

(cl:ensure-generic-function 'm-val :lambda-list '(m))
(cl:defmethod m-val ((m <IsDoneBoltTightening-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:m-val is deprecated.  Use dsr_msgs-srv:m instead.")
  (m m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <IsDoneBoltTightening-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:timeout-val is deprecated.  Use dsr_msgs-srv:timeout instead.")
  (timeout m))

(cl:ensure-generic-function 'axis-val :lambda-list '(m))
(cl:defmethod axis-val ((m <IsDoneBoltTightening-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:axis-val is deprecated.  Use dsr_msgs-srv:axis instead.")
  (axis m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IsDoneBoltTightening-request>) ostream)
  "Serializes a message object of type '<IsDoneBoltTightening-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timeout))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'axis)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IsDoneBoltTightening-request>) istream)
  "Deserializes a message object of type '<IsDoneBoltTightening-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timeout) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'axis) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IsDoneBoltTightening-request>)))
  "Returns string type for a service object of type '<IsDoneBoltTightening-request>"
  "dsr_msgs/IsDoneBoltTighteningRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsDoneBoltTightening-request)))
  "Returns string type for a service object of type 'IsDoneBoltTightening-request"
  "dsr_msgs/IsDoneBoltTighteningRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IsDoneBoltTightening-request>)))
  "Returns md5sum for a message object of type '<IsDoneBoltTightening-request>"
  "be84affb8d741115ea457a393a21b8b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IsDoneBoltTightening-request)))
  "Returns md5sum for a message object of type 'IsDoneBoltTightening-request"
  "be84affb8d741115ea457a393a21b8b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IsDoneBoltTightening-request>)))
  "Returns full string definition for message of type '<IsDoneBoltTightening-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# is_done_bolt_tightening  ~%#____________________________________________________________________________________________~%~%float64    m                 # Target torque  ~%float64    timeout           # Monitoring duration [sec]  ~%int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IsDoneBoltTightening-request)))
  "Returns full string definition for message of type 'IsDoneBoltTightening-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# is_done_bolt_tightening  ~%#____________________________________________________________________________________________~%~%float64    m                 # Target torque  ~%float64    timeout           # Monitoring duration [sec]  ~%int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IsDoneBoltTightening-request>))
  (cl:+ 0
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IsDoneBoltTightening-request>))
  "Converts a ROS message object to a list"
  (cl:list 'IsDoneBoltTightening-request
    (cl:cons ':m (m msg))
    (cl:cons ':timeout (timeout msg))
    (cl:cons ':axis (axis msg))
))
;//! \htmlinclude IsDoneBoltTightening-response.msg.html

(cl:defclass <IsDoneBoltTightening-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass IsDoneBoltTightening-response (<IsDoneBoltTightening-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IsDoneBoltTightening-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IsDoneBoltTightening-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<IsDoneBoltTightening-response> is deprecated: use dsr_msgs-srv:IsDoneBoltTightening-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <IsDoneBoltTightening-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IsDoneBoltTightening-response>) ostream)
  "Serializes a message object of type '<IsDoneBoltTightening-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IsDoneBoltTightening-response>) istream)
  "Deserializes a message object of type '<IsDoneBoltTightening-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IsDoneBoltTightening-response>)))
  "Returns string type for a service object of type '<IsDoneBoltTightening-response>"
  "dsr_msgs/IsDoneBoltTighteningResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsDoneBoltTightening-response)))
  "Returns string type for a service object of type 'IsDoneBoltTightening-response"
  "dsr_msgs/IsDoneBoltTighteningResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IsDoneBoltTightening-response>)))
  "Returns md5sum for a message object of type '<IsDoneBoltTightening-response>"
  "be84affb8d741115ea457a393a21b8b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IsDoneBoltTightening-response)))
  "Returns md5sum for a message object of type 'IsDoneBoltTightening-response"
  "be84affb8d741115ea457a393a21b8b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IsDoneBoltTightening-response>)))
  "Returns full string definition for message of type '<IsDoneBoltTightening-response>"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IsDoneBoltTightening-response)))
  "Returns full string definition for message of type 'IsDoneBoltTightening-response"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IsDoneBoltTightening-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IsDoneBoltTightening-response>))
  "Converts a ROS message object to a list"
  (cl:list 'IsDoneBoltTightening-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'IsDoneBoltTightening)))
  'IsDoneBoltTightening-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'IsDoneBoltTightening)))
  'IsDoneBoltTightening-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IsDoneBoltTightening)))
  "Returns string type for a service object of type '<IsDoneBoltTightening>"
  "dsr_msgs/IsDoneBoltTightening")