; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetToolDigitalOutput-request.msg.html

(cl:defclass <GetToolDigitalOutput-request> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetToolDigitalOutput-request (<GetToolDigitalOutput-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetToolDigitalOutput-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetToolDigitalOutput-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetToolDigitalOutput-request> is deprecated: use dsr_msgs-srv:GetToolDigitalOutput-request instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <GetToolDigitalOutput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:index-val is deprecated.  Use dsr_msgs-srv:index instead.")
  (index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetToolDigitalOutput-request>) ostream)
  "Serializes a message object of type '<GetToolDigitalOutput-request>"
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetToolDigitalOutput-request>) istream)
  "Deserializes a message object of type '<GetToolDigitalOutput-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetToolDigitalOutput-request>)))
  "Returns string type for a service object of type '<GetToolDigitalOutput-request>"
  "dsr_msgs/GetToolDigitalOutputRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetToolDigitalOutput-request)))
  "Returns string type for a service object of type 'GetToolDigitalOutput-request"
  "dsr_msgs/GetToolDigitalOutputRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetToolDigitalOutput-request>)))
  "Returns md5sum for a message object of type '<GetToolDigitalOutput-request>"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetToolDigitalOutput-request)))
  "Returns md5sum for a message object of type 'GetToolDigitalOutput-request"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetToolDigitalOutput-request>)))
  "Returns full string definition for message of type '<GetToolDigitalOutput-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_tool_digital_output  ~%# This service gets the current tool io output status.~%#____________________________________________________________________________________________~%~%int8       index   # flange digital output port(1 ~~ 6)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetToolDigitalOutput-request)))
  "Returns full string definition for message of type 'GetToolDigitalOutput-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_tool_digital_output  ~%# This service gets the current tool io output status.~%#____________________________________________________________________________________________~%~%int8       index   # flange digital output port(1 ~~ 6)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetToolDigitalOutput-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetToolDigitalOutput-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetToolDigitalOutput-request
    (cl:cons ':index (index msg))
))
;//! \htmlinclude GetToolDigitalOutput-response.msg.html

(cl:defclass <GetToolDigitalOutput-response> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetToolDigitalOutput-response (<GetToolDigitalOutput-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetToolDigitalOutput-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetToolDigitalOutput-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetToolDigitalOutput-response> is deprecated: use dsr_msgs-srv:GetToolDigitalOutput-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <GetToolDigitalOutput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:value-val is deprecated.  Use dsr_msgs-srv:value instead.")
  (value m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetToolDigitalOutput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetToolDigitalOutput-response>) ostream)
  "Serializes a message object of type '<GetToolDigitalOutput-response>"
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetToolDigitalOutput-response>) istream)
  "Deserializes a message object of type '<GetToolDigitalOutput-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetToolDigitalOutput-response>)))
  "Returns string type for a service object of type '<GetToolDigitalOutput-response>"
  "dsr_msgs/GetToolDigitalOutputResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetToolDigitalOutput-response)))
  "Returns string type for a service object of type 'GetToolDigitalOutput-response"
  "dsr_msgs/GetToolDigitalOutputResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetToolDigitalOutput-response>)))
  "Returns md5sum for a message object of type '<GetToolDigitalOutput-response>"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetToolDigitalOutput-response)))
  "Returns md5sum for a message object of type 'GetToolDigitalOutput-response"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetToolDigitalOutput-response>)))
  "Returns full string definition for message of type '<GetToolDigitalOutput-response>"
  (cl:format cl:nil "int8       value   # Current output status (0 : ON, 1 : OFF)~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetToolDigitalOutput-response)))
  "Returns full string definition for message of type 'GetToolDigitalOutput-response"
  (cl:format cl:nil "int8       value   # Current output status (0 : ON, 1 : OFF)~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetToolDigitalOutput-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetToolDigitalOutput-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetToolDigitalOutput-response
    (cl:cons ':value (value msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetToolDigitalOutput)))
  'GetToolDigitalOutput-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetToolDigitalOutput)))
  'GetToolDigitalOutput-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetToolDigitalOutput)))
  "Returns string type for a service object of type '<GetToolDigitalOutput>"
  "dsr_msgs/GetToolDigitalOutput")