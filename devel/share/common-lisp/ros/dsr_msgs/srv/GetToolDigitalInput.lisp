; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetToolDigitalInput-request.msg.html

(cl:defclass <GetToolDigitalInput-request> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetToolDigitalInput-request (<GetToolDigitalInput-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetToolDigitalInput-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetToolDigitalInput-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetToolDigitalInput-request> is deprecated: use dsr_msgs-srv:GetToolDigitalInput-request instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <GetToolDigitalInput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:index-val is deprecated.  Use dsr_msgs-srv:index instead.")
  (index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetToolDigitalInput-request>) ostream)
  "Serializes a message object of type '<GetToolDigitalInput-request>"
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetToolDigitalInput-request>) istream)
  "Deserializes a message object of type '<GetToolDigitalInput-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetToolDigitalInput-request>)))
  "Returns string type for a service object of type '<GetToolDigitalInput-request>"
  "dsr_msgs/GetToolDigitalInputRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetToolDigitalInput-request)))
  "Returns string type for a service object of type 'GetToolDigitalInput-request"
  "dsr_msgs/GetToolDigitalInputRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetToolDigitalInput-request>)))
  "Returns md5sum for a message object of type '<GetToolDigitalInput-request>"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetToolDigitalInput-request)))
  "Returns md5sum for a message object of type 'GetToolDigitalInput-request"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetToolDigitalInput-request>)))
  "Returns full string definition for message of type '<GetToolDigitalInput-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_tool_digital_input  ~%# This service gets the current controlbox io output status.~%#____________________________________________________________________________________________~%~%int8        index    # Digital Input in Flange(1 ~~ 6) ~%                     # <GPIO_TOOL_DIGITAL_INDEX> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetToolDigitalInput-request)))
  "Returns full string definition for message of type 'GetToolDigitalInput-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_tool_digital_input  ~%# This service gets the current controlbox io output status.~%#____________________________________________________________________________________________~%~%int8        index    # Digital Input in Flange(1 ~~ 6) ~%                     # <GPIO_TOOL_DIGITAL_INDEX> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetToolDigitalInput-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetToolDigitalInput-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetToolDigitalInput-request
    (cl:cons ':index (index msg))
))
;//! \htmlinclude GetToolDigitalInput-response.msg.html

(cl:defclass <GetToolDigitalInput-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetToolDigitalInput-response (<GetToolDigitalInput-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetToolDigitalInput-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetToolDigitalInput-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetToolDigitalInput-response> is deprecated: use dsr_msgs-srv:GetToolDigitalInput-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <GetToolDigitalInput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:value-val is deprecated.  Use dsr_msgs-srv:value instead.")
  (value m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetToolDigitalInput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetToolDigitalInput-response>) ostream)
  "Serializes a message object of type '<GetToolDigitalInput-response>"
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetToolDigitalInput-response>) istream)
  "Deserializes a message object of type '<GetToolDigitalInput-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetToolDigitalInput-response>)))
  "Returns string type for a service object of type '<GetToolDigitalInput-response>"
  "dsr_msgs/GetToolDigitalInputResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetToolDigitalInput-response)))
  "Returns string type for a service object of type 'GetToolDigitalInput-response"
  "dsr_msgs/GetToolDigitalInputResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetToolDigitalInput-response>)))
  "Returns md5sum for a message object of type '<GetToolDigitalInput-response>"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetToolDigitalInput-response)))
  "Returns md5sum for a message object of type 'GetToolDigitalInput-response"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetToolDigitalInput-response>)))
  "Returns full string definition for message of type '<GetToolDigitalInput-response>"
  (cl:format cl:nil "int8        value    # 0=OFF, 1=ON~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetToolDigitalInput-response)))
  "Returns full string definition for message of type 'GetToolDigitalInput-response"
  (cl:format cl:nil "int8        value    # 0=OFF, 1=ON~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetToolDigitalInput-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetToolDigitalInput-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetToolDigitalInput-response
    (cl:cons ':value (value msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetToolDigitalInput)))
  'GetToolDigitalInput-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetToolDigitalInput)))
  'GetToolDigitalInput-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetToolDigitalInput)))
  "Returns string type for a service object of type '<GetToolDigitalInput>"
  "dsr_msgs/GetToolDigitalInput")