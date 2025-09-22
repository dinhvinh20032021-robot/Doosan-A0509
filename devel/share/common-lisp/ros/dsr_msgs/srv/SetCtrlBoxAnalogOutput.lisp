; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetCtrlBoxAnalogOutput-request.msg.html

(cl:defclass <SetCtrlBoxAnalogOutput-request> (roslisp-msg-protocol:ros-message)
  ((channel
    :reader channel
    :initarg :channel
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetCtrlBoxAnalogOutput-request (<SetCtrlBoxAnalogOutput-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCtrlBoxAnalogOutput-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCtrlBoxAnalogOutput-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetCtrlBoxAnalogOutput-request> is deprecated: use dsr_msgs-srv:SetCtrlBoxAnalogOutput-request instead.")))

(cl:ensure-generic-function 'channel-val :lambda-list '(m))
(cl:defmethod channel-val ((m <SetCtrlBoxAnalogOutput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:channel-val is deprecated.  Use dsr_msgs-srv:channel instead.")
  (channel m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SetCtrlBoxAnalogOutput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:value-val is deprecated.  Use dsr_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCtrlBoxAnalogOutput-request>) ostream)
  "Serializes a message object of type '<SetCtrlBoxAnalogOutput-request>"
  (cl:let* ((signed (cl:slot-value msg 'channel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCtrlBoxAnalogOutput-request>) istream)
  "Deserializes a message object of type '<SetCtrlBoxAnalogOutput-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'channel) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCtrlBoxAnalogOutput-request>)))
  "Returns string type for a service object of type '<SetCtrlBoxAnalogOutput-request>"
  "dsr_msgs/SetCtrlBoxAnalogOutputRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCtrlBoxAnalogOutput-request)))
  "Returns string type for a service object of type 'SetCtrlBoxAnalogOutput-request"
  "dsr_msgs/SetCtrlBoxAnalogOutputRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCtrlBoxAnalogOutput-request>)))
  "Returns md5sum for a message object of type '<SetCtrlBoxAnalogOutput-request>"
  "bb0effb74873abe56de0ddcef9d502cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCtrlBoxAnalogOutput-request)))
  "Returns md5sum for a message object of type 'SetCtrlBoxAnalogOutput-request"
  "bb0effb74873abe56de0ddcef9d502cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCtrlBoxAnalogOutput-request>)))
  "Returns full string definition for message of type '<SetCtrlBoxAnalogOutput-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_ctrl_box_analog_output  ~%#____________________________________________________________________________________________~%~%int8        channel  # 1 = ch1, 2= ch2 ~%float64     value   #~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCtrlBoxAnalogOutput-request)))
  "Returns full string definition for message of type 'SetCtrlBoxAnalogOutput-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_ctrl_box_analog_output  ~%#____________________________________________________________________________________________~%~%int8        channel  # 1 = ch1, 2= ch2 ~%float64     value   #~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCtrlBoxAnalogOutput-request>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCtrlBoxAnalogOutput-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCtrlBoxAnalogOutput-request
    (cl:cons ':channel (channel msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude SetCtrlBoxAnalogOutput-response.msg.html

(cl:defclass <SetCtrlBoxAnalogOutput-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetCtrlBoxAnalogOutput-response (<SetCtrlBoxAnalogOutput-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCtrlBoxAnalogOutput-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCtrlBoxAnalogOutput-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetCtrlBoxAnalogOutput-response> is deprecated: use dsr_msgs-srv:SetCtrlBoxAnalogOutput-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetCtrlBoxAnalogOutput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCtrlBoxAnalogOutput-response>) ostream)
  "Serializes a message object of type '<SetCtrlBoxAnalogOutput-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCtrlBoxAnalogOutput-response>) istream)
  "Deserializes a message object of type '<SetCtrlBoxAnalogOutput-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCtrlBoxAnalogOutput-response>)))
  "Returns string type for a service object of type '<SetCtrlBoxAnalogOutput-response>"
  "dsr_msgs/SetCtrlBoxAnalogOutputResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCtrlBoxAnalogOutput-response)))
  "Returns string type for a service object of type 'SetCtrlBoxAnalogOutput-response"
  "dsr_msgs/SetCtrlBoxAnalogOutputResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCtrlBoxAnalogOutput-response>)))
  "Returns md5sum for a message object of type '<SetCtrlBoxAnalogOutput-response>"
  "bb0effb74873abe56de0ddcef9d502cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCtrlBoxAnalogOutput-response)))
  "Returns md5sum for a message object of type 'SetCtrlBoxAnalogOutput-response"
  "bb0effb74873abe56de0ddcef9d502cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCtrlBoxAnalogOutput-response>)))
  "Returns full string definition for message of type '<SetCtrlBoxAnalogOutput-response>"
  (cl:format cl:nil "bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCtrlBoxAnalogOutput-response)))
  "Returns full string definition for message of type 'SetCtrlBoxAnalogOutput-response"
  (cl:format cl:nil "bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCtrlBoxAnalogOutput-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCtrlBoxAnalogOutput-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCtrlBoxAnalogOutput-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetCtrlBoxAnalogOutput)))
  'SetCtrlBoxAnalogOutput-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetCtrlBoxAnalogOutput)))
  'SetCtrlBoxAnalogOutput-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCtrlBoxAnalogOutput)))
  "Returns string type for a service object of type '<SetCtrlBoxAnalogOutput>"
  "dsr_msgs/SetCtrlBoxAnalogOutput")