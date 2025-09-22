; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetCtrlBoxDigitalInput-request.msg.html

(cl:defclass <GetCtrlBoxDigitalInput-request> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetCtrlBoxDigitalInput-request (<GetCtrlBoxDigitalInput-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCtrlBoxDigitalInput-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCtrlBoxDigitalInput-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCtrlBoxDigitalInput-request> is deprecated: use dsr_msgs-srv:GetCtrlBoxDigitalInput-request instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <GetCtrlBoxDigitalInput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:index-val is deprecated.  Use dsr_msgs-srv:index instead.")
  (index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCtrlBoxDigitalInput-request>) ostream)
  "Serializes a message object of type '<GetCtrlBoxDigitalInput-request>"
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCtrlBoxDigitalInput-request>) istream)
  "Deserializes a message object of type '<GetCtrlBoxDigitalInput-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCtrlBoxDigitalInput-request>)))
  "Returns string type for a service object of type '<GetCtrlBoxDigitalInput-request>"
  "dsr_msgs/GetCtrlBoxDigitalInputRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCtrlBoxDigitalInput-request)))
  "Returns string type for a service object of type 'GetCtrlBoxDigitalInput-request"
  "dsr_msgs/GetCtrlBoxDigitalInputRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCtrlBoxDigitalInput-request>)))
  "Returns md5sum for a message object of type '<GetCtrlBoxDigitalInput-request>"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCtrlBoxDigitalInput-request)))
  "Returns md5sum for a message object of type 'GetCtrlBoxDigitalInput-request"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCtrlBoxDigitalInput-request>)))
  "Returns full string definition for message of type '<GetCtrlBoxDigitalInput-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_digital_input~%# This service reads the I/O signals from digital contact points of the controller and reads the digital input contact value.~%#____________________________________________________________________________________________~%~%int8        index    # Digital Input in Control Box(1 ~~ 16) ~%                     # <GPIO_CTRLBOX_DIGITAL_INDEX>~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCtrlBoxDigitalInput-request)))
  "Returns full string definition for message of type 'GetCtrlBoxDigitalInput-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_digital_input~%# This service reads the I/O signals from digital contact points of the controller and reads the digital input contact value.~%#____________________________________________________________________________________________~%~%int8        index    # Digital Input in Control Box(1 ~~ 16) ~%                     # <GPIO_CTRLBOX_DIGITAL_INDEX>~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCtrlBoxDigitalInput-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCtrlBoxDigitalInput-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCtrlBoxDigitalInput-request
    (cl:cons ':index (index msg))
))
;//! \htmlinclude GetCtrlBoxDigitalInput-response.msg.html

(cl:defclass <GetCtrlBoxDigitalInput-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetCtrlBoxDigitalInput-response (<GetCtrlBoxDigitalInput-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCtrlBoxDigitalInput-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCtrlBoxDigitalInput-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCtrlBoxDigitalInput-response> is deprecated: use dsr_msgs-srv:GetCtrlBoxDigitalInput-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <GetCtrlBoxDigitalInput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:value-val is deprecated.  Use dsr_msgs-srv:value instead.")
  (value m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetCtrlBoxDigitalInput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCtrlBoxDigitalInput-response>) ostream)
  "Serializes a message object of type '<GetCtrlBoxDigitalInput-response>"
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCtrlBoxDigitalInput-response>) istream)
  "Deserializes a message object of type '<GetCtrlBoxDigitalInput-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCtrlBoxDigitalInput-response>)))
  "Returns string type for a service object of type '<GetCtrlBoxDigitalInput-response>"
  "dsr_msgs/GetCtrlBoxDigitalInputResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCtrlBoxDigitalInput-response)))
  "Returns string type for a service object of type 'GetCtrlBoxDigitalInput-response"
  "dsr_msgs/GetCtrlBoxDigitalInputResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCtrlBoxDigitalInput-response>)))
  "Returns md5sum for a message object of type '<GetCtrlBoxDigitalInput-response>"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCtrlBoxDigitalInput-response)))
  "Returns md5sum for a message object of type 'GetCtrlBoxDigitalInput-response"
  "66249e10e881902f2908d5046d836788")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCtrlBoxDigitalInput-response>)))
  "Returns full string definition for message of type '<GetCtrlBoxDigitalInput-response>"
  (cl:format cl:nil "int8        value    # 0=OFF, 1=ON~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCtrlBoxDigitalInput-response)))
  "Returns full string definition for message of type 'GetCtrlBoxDigitalInput-response"
  (cl:format cl:nil "int8        value    # 0=OFF, 1=ON~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCtrlBoxDigitalInput-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCtrlBoxDigitalInput-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCtrlBoxDigitalInput-response
    (cl:cons ':value (value msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCtrlBoxDigitalInput)))
  'GetCtrlBoxDigitalInput-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCtrlBoxDigitalInput)))
  'GetCtrlBoxDigitalInput-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCtrlBoxDigitalInput)))
  "Returns string type for a service object of type '<GetCtrlBoxDigitalInput>"
  "dsr_msgs/GetCtrlBoxDigitalInput")