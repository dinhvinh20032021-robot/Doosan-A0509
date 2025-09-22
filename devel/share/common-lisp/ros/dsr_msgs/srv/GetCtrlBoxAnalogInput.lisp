; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetCtrlBoxAnalogInput-request.msg.html

(cl:defclass <GetCtrlBoxAnalogInput-request> (roslisp-msg-protocol:ros-message)
  ((channel
    :reader channel
    :initarg :channel
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetCtrlBoxAnalogInput-request (<GetCtrlBoxAnalogInput-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCtrlBoxAnalogInput-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCtrlBoxAnalogInput-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCtrlBoxAnalogInput-request> is deprecated: use dsr_msgs-srv:GetCtrlBoxAnalogInput-request instead.")))

(cl:ensure-generic-function 'channel-val :lambda-list '(m))
(cl:defmethod channel-val ((m <GetCtrlBoxAnalogInput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:channel-val is deprecated.  Use dsr_msgs-srv:channel instead.")
  (channel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCtrlBoxAnalogInput-request>) ostream)
  "Serializes a message object of type '<GetCtrlBoxAnalogInput-request>"
  (cl:let* ((signed (cl:slot-value msg 'channel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCtrlBoxAnalogInput-request>) istream)
  "Deserializes a message object of type '<GetCtrlBoxAnalogInput-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'channel) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCtrlBoxAnalogInput-request>)))
  "Returns string type for a service object of type '<GetCtrlBoxAnalogInput-request>"
  "dsr_msgs/GetCtrlBoxAnalogInputRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCtrlBoxAnalogInput-request)))
  "Returns string type for a service object of type 'GetCtrlBoxAnalogInput-request"
  "dsr_msgs/GetCtrlBoxAnalogInputRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCtrlBoxAnalogInput-request>)))
  "Returns md5sum for a message object of type '<GetCtrlBoxAnalogInput-request>"
  "bfe8574a1267a2cf39f4166121f897f2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCtrlBoxAnalogInput-request)))
  "Returns md5sum for a message object of type 'GetCtrlBoxAnalogInput-request"
  "bfe8574a1267a2cf39f4166121f897f2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCtrlBoxAnalogInput-request>)))
  "Returns full string definition for message of type '<GetCtrlBoxAnalogInput-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_analog_input ~%#____________________________________________________________________________________________~%~%int8        channel    # 1 = ch1, 2= ch2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCtrlBoxAnalogInput-request)))
  "Returns full string definition for message of type 'GetCtrlBoxAnalogInput-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_analog_input ~%#____________________________________________________________________________________________~%~%int8        channel    # 1 = ch1, 2= ch2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCtrlBoxAnalogInput-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCtrlBoxAnalogInput-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCtrlBoxAnalogInput-request
    (cl:cons ':channel (channel msg))
))
;//! \htmlinclude GetCtrlBoxAnalogInput-response.msg.html

(cl:defclass <GetCtrlBoxAnalogInput-response> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetCtrlBoxAnalogInput-response (<GetCtrlBoxAnalogInput-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCtrlBoxAnalogInput-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCtrlBoxAnalogInput-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCtrlBoxAnalogInput-response> is deprecated: use dsr_msgs-srv:GetCtrlBoxAnalogInput-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <GetCtrlBoxAnalogInput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:value-val is deprecated.  Use dsr_msgs-srv:value instead.")
  (value m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetCtrlBoxAnalogInput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCtrlBoxAnalogInput-response>) ostream)
  "Serializes a message object of type '<GetCtrlBoxAnalogInput-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCtrlBoxAnalogInput-response>) istream)
  "Deserializes a message object of type '<GetCtrlBoxAnalogInput-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCtrlBoxAnalogInput-response>)))
  "Returns string type for a service object of type '<GetCtrlBoxAnalogInput-response>"
  "dsr_msgs/GetCtrlBoxAnalogInputResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCtrlBoxAnalogInput-response)))
  "Returns string type for a service object of type 'GetCtrlBoxAnalogInput-response"
  "dsr_msgs/GetCtrlBoxAnalogInputResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCtrlBoxAnalogInput-response>)))
  "Returns md5sum for a message object of type '<GetCtrlBoxAnalogInput-response>"
  "bfe8574a1267a2cf39f4166121f897f2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCtrlBoxAnalogInput-response)))
  "Returns md5sum for a message object of type 'GetCtrlBoxAnalogInput-response"
  "bfe8574a1267a2cf39f4166121f897f2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCtrlBoxAnalogInput-response>)))
  "Returns full string definition for message of type '<GetCtrlBoxAnalogInput-response>"
  (cl:format cl:nil "float64     value~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCtrlBoxAnalogInput-response)))
  "Returns full string definition for message of type 'GetCtrlBoxAnalogInput-response"
  (cl:format cl:nil "float64     value~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCtrlBoxAnalogInput-response>))
  (cl:+ 0
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCtrlBoxAnalogInput-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCtrlBoxAnalogInput-response
    (cl:cons ':value (value msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCtrlBoxAnalogInput)))
  'GetCtrlBoxAnalogInput-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCtrlBoxAnalogInput)))
  'GetCtrlBoxAnalogInput-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCtrlBoxAnalogInput)))
  "Returns string type for a service object of type '<GetCtrlBoxAnalogInput>"
  "dsr_msgs/GetCtrlBoxAnalogInput")