; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetToolDigitalOutput-request.msg.html

(cl:defclass <SetToolDigitalOutput-request> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetToolDigitalOutput-request (<SetToolDigitalOutput-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetToolDigitalOutput-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetToolDigitalOutput-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetToolDigitalOutput-request> is deprecated: use dsr_msgs-srv:SetToolDigitalOutput-request instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <SetToolDigitalOutput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:index-val is deprecated.  Use dsr_msgs-srv:index instead.")
  (index m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SetToolDigitalOutput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:value-val is deprecated.  Use dsr_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetToolDigitalOutput-request>) ostream)
  "Serializes a message object of type '<SetToolDigitalOutput-request>"
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetToolDigitalOutput-request>) istream)
  "Deserializes a message object of type '<SetToolDigitalOutput-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetToolDigitalOutput-request>)))
  "Returns string type for a service object of type '<SetToolDigitalOutput-request>"
  "dsr_msgs/SetToolDigitalOutputRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetToolDigitalOutput-request)))
  "Returns string type for a service object of type 'SetToolDigitalOutput-request"
  "dsr_msgs/SetToolDigitalOutputRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetToolDigitalOutput-request>)))
  "Returns md5sum for a message object of type '<SetToolDigitalOutput-request>"
  "fb5dce2e6988e39a422dd00d9d97c412")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetToolDigitalOutput-request)))
  "Returns md5sum for a message object of type 'SetToolDigitalOutput-request"
  "fb5dce2e6988e39a422dd00d9d97c412")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetToolDigitalOutput-request>)))
  "Returns full string definition for message of type '<SetToolDigitalOutput-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_tool_digital_output  ~%# This service sends ouptput to tool io.~%#____________________________________________________________________________________________~%~%int8       index   # flange digital output port(1 ~~ 6)~%int8       value   # 0 : ON, 1 : OFF~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetToolDigitalOutput-request)))
  "Returns full string definition for message of type 'SetToolDigitalOutput-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_tool_digital_output  ~%# This service sends ouptput to tool io.~%#____________________________________________________________________________________________~%~%int8       index   # flange digital output port(1 ~~ 6)~%int8       value   # 0 : ON, 1 : OFF~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetToolDigitalOutput-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetToolDigitalOutput-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetToolDigitalOutput-request
    (cl:cons ':index (index msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude SetToolDigitalOutput-response.msg.html

(cl:defclass <SetToolDigitalOutput-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetToolDigitalOutput-response (<SetToolDigitalOutput-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetToolDigitalOutput-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetToolDigitalOutput-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetToolDigitalOutput-response> is deprecated: use dsr_msgs-srv:SetToolDigitalOutput-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetToolDigitalOutput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetToolDigitalOutput-response>) ostream)
  "Serializes a message object of type '<SetToolDigitalOutput-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetToolDigitalOutput-response>) istream)
  "Deserializes a message object of type '<SetToolDigitalOutput-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetToolDigitalOutput-response>)))
  "Returns string type for a service object of type '<SetToolDigitalOutput-response>"
  "dsr_msgs/SetToolDigitalOutputResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetToolDigitalOutput-response)))
  "Returns string type for a service object of type 'SetToolDigitalOutput-response"
  "dsr_msgs/SetToolDigitalOutputResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetToolDigitalOutput-response>)))
  "Returns md5sum for a message object of type '<SetToolDigitalOutput-response>"
  "fb5dce2e6988e39a422dd00d9d97c412")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetToolDigitalOutput-response)))
  "Returns md5sum for a message object of type 'SetToolDigitalOutput-response"
  "fb5dce2e6988e39a422dd00d9d97c412")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetToolDigitalOutput-response>)))
  "Returns full string definition for message of type '<SetToolDigitalOutput-response>"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetToolDigitalOutput-response)))
  "Returns full string definition for message of type 'SetToolDigitalOutput-response"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetToolDigitalOutput-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetToolDigitalOutput-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetToolDigitalOutput-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetToolDigitalOutput)))
  'SetToolDigitalOutput-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetToolDigitalOutput)))
  'SetToolDigitalOutput-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetToolDigitalOutput)))
  "Returns string type for a service object of type '<SetToolDigitalOutput>"
  "dsr_msgs/SetToolDigitalOutput")