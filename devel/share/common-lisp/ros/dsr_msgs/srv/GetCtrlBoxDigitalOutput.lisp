; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetCtrlBoxDigitalOutput-request.msg.html

(cl:defclass <GetCtrlBoxDigitalOutput-request> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetCtrlBoxDigitalOutput-request (<GetCtrlBoxDigitalOutput-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCtrlBoxDigitalOutput-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCtrlBoxDigitalOutput-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCtrlBoxDigitalOutput-request> is deprecated: use dsr_msgs-srv:GetCtrlBoxDigitalOutput-request instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <GetCtrlBoxDigitalOutput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:index-val is deprecated.  Use dsr_msgs-srv:index instead.")
  (index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCtrlBoxDigitalOutput-request>) ostream)
  "Serializes a message object of type '<GetCtrlBoxDigitalOutput-request>"
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCtrlBoxDigitalOutput-request>) istream)
  "Deserializes a message object of type '<GetCtrlBoxDigitalOutput-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCtrlBoxDigitalOutput-request>)))
  "Returns string type for a service object of type '<GetCtrlBoxDigitalOutput-request>"
  "dsr_msgs/GetCtrlBoxDigitalOutputRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCtrlBoxDigitalOutput-request)))
  "Returns string type for a service object of type 'GetCtrlBoxDigitalOutput-request"
  "dsr_msgs/GetCtrlBoxDigitalOutputRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCtrlBoxDigitalOutput-request>)))
  "Returns md5sum for a message object of type '<GetCtrlBoxDigitalOutput-request>"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCtrlBoxDigitalOutput-request)))
  "Returns md5sum for a message object of type 'GetCtrlBoxDigitalOutput-request"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCtrlBoxDigitalOutput-request>)))
  "Returns full string definition for message of type '<GetCtrlBoxDigitalOutput-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_digital_output  ~%#____________________________________________________________________________________________~%~%int8       index    # ctrlbox digital output port(1 ~~ 16)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCtrlBoxDigitalOutput-request)))
  "Returns full string definition for message of type 'GetCtrlBoxDigitalOutput-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_digital_output  ~%#____________________________________________________________________________________________~%~%int8       index    # ctrlbox digital output port(1 ~~ 16)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCtrlBoxDigitalOutput-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCtrlBoxDigitalOutput-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCtrlBoxDigitalOutput-request
    (cl:cons ':index (index msg))
))
;//! \htmlinclude GetCtrlBoxDigitalOutput-response.msg.html

(cl:defclass <GetCtrlBoxDigitalOutput-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetCtrlBoxDigitalOutput-response (<GetCtrlBoxDigitalOutput-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCtrlBoxDigitalOutput-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCtrlBoxDigitalOutput-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCtrlBoxDigitalOutput-response> is deprecated: use dsr_msgs-srv:GetCtrlBoxDigitalOutput-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <GetCtrlBoxDigitalOutput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:value-val is deprecated.  Use dsr_msgs-srv:value instead.")
  (value m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetCtrlBoxDigitalOutput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCtrlBoxDigitalOutput-response>) ostream)
  "Serializes a message object of type '<GetCtrlBoxDigitalOutput-response>"
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCtrlBoxDigitalOutput-response>) istream)
  "Deserializes a message object of type '<GetCtrlBoxDigitalOutput-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCtrlBoxDigitalOutput-response>)))
  "Returns string type for a service object of type '<GetCtrlBoxDigitalOutput-response>"
  "dsr_msgs/GetCtrlBoxDigitalOutputResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCtrlBoxDigitalOutput-response)))
  "Returns string type for a service object of type 'GetCtrlBoxDigitalOutput-response"
  "dsr_msgs/GetCtrlBoxDigitalOutputResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCtrlBoxDigitalOutput-response>)))
  "Returns md5sum for a message object of type '<GetCtrlBoxDigitalOutput-response>"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCtrlBoxDigitalOutput-response)))
  "Returns md5sum for a message object of type 'GetCtrlBoxDigitalOutput-response"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCtrlBoxDigitalOutput-response>)))
  "Returns full string definition for message of type '<GetCtrlBoxDigitalOutput-response>"
  (cl:format cl:nil "int8       value    # Current output status (0 : ON, 1 : OFF)~%bool       success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCtrlBoxDigitalOutput-response)))
  "Returns full string definition for message of type 'GetCtrlBoxDigitalOutput-response"
  (cl:format cl:nil "int8       value    # Current output status (0 : ON, 1 : OFF)~%bool       success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCtrlBoxDigitalOutput-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCtrlBoxDigitalOutput-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCtrlBoxDigitalOutput-response
    (cl:cons ':value (value msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCtrlBoxDigitalOutput)))
  'GetCtrlBoxDigitalOutput-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCtrlBoxDigitalOutput)))
  'GetCtrlBoxDigitalOutput-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCtrlBoxDigitalOutput)))
  "Returns string type for a service object of type '<GetCtrlBoxDigitalOutput>"
  "dsr_msgs/GetCtrlBoxDigitalOutput")