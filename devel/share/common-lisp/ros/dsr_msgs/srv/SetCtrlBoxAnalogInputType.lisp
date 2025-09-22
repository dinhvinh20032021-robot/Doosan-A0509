; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetCtrlBoxAnalogInputType-request.msg.html

(cl:defclass <SetCtrlBoxAnalogInputType-request> (roslisp-msg-protocol:ros-message)
  ((channel
    :reader channel
    :initarg :channel
    :type cl:fixnum
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetCtrlBoxAnalogInputType-request (<SetCtrlBoxAnalogInputType-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCtrlBoxAnalogInputType-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCtrlBoxAnalogInputType-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetCtrlBoxAnalogInputType-request> is deprecated: use dsr_msgs-srv:SetCtrlBoxAnalogInputType-request instead.")))

(cl:ensure-generic-function 'channel-val :lambda-list '(m))
(cl:defmethod channel-val ((m <SetCtrlBoxAnalogInputType-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:channel-val is deprecated.  Use dsr_msgs-srv:channel instead.")
  (channel m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <SetCtrlBoxAnalogInputType-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:mode-val is deprecated.  Use dsr_msgs-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCtrlBoxAnalogInputType-request>) ostream)
  "Serializes a message object of type '<SetCtrlBoxAnalogInputType-request>"
  (cl:let* ((signed (cl:slot-value msg 'channel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCtrlBoxAnalogInputType-request>) istream)
  "Deserializes a message object of type '<SetCtrlBoxAnalogInputType-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'channel) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCtrlBoxAnalogInputType-request>)))
  "Returns string type for a service object of type '<SetCtrlBoxAnalogInputType-request>"
  "dsr_msgs/SetCtrlBoxAnalogInputTypeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCtrlBoxAnalogInputType-request)))
  "Returns string type for a service object of type 'SetCtrlBoxAnalogInputType-request"
  "dsr_msgs/SetCtrlBoxAnalogInputTypeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCtrlBoxAnalogInputType-request>)))
  "Returns md5sum for a message object of type '<SetCtrlBoxAnalogInputType-request>"
  "edb11f75523beab953b2183675ee124d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCtrlBoxAnalogInputType-request)))
  "Returns md5sum for a message object of type 'SetCtrlBoxAnalogInputType-request"
  "edb11f75523beab953b2183675ee124d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCtrlBoxAnalogInputType-request>)))
  "Returns full string definition for message of type '<SetCtrlBoxAnalogInputType-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_ctrl_box_analog_input_type  ~%#____________________________________________________________________________________________~%~%int8        channel  # 1 = ch1, 2= ch2 ~%int8        mode     # 0 = current, 1 = voltage~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCtrlBoxAnalogInputType-request)))
  "Returns full string definition for message of type 'SetCtrlBoxAnalogInputType-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_ctrl_box_analog_input_type  ~%#____________________________________________________________________________________________~%~%int8        channel  # 1 = ch1, 2= ch2 ~%int8        mode     # 0 = current, 1 = voltage~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCtrlBoxAnalogInputType-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCtrlBoxAnalogInputType-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCtrlBoxAnalogInputType-request
    (cl:cons ':channel (channel msg))
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude SetCtrlBoxAnalogInputType-response.msg.html

(cl:defclass <SetCtrlBoxAnalogInputType-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetCtrlBoxAnalogInputType-response (<SetCtrlBoxAnalogInputType-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCtrlBoxAnalogInputType-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCtrlBoxAnalogInputType-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetCtrlBoxAnalogInputType-response> is deprecated: use dsr_msgs-srv:SetCtrlBoxAnalogInputType-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetCtrlBoxAnalogInputType-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCtrlBoxAnalogInputType-response>) ostream)
  "Serializes a message object of type '<SetCtrlBoxAnalogInputType-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCtrlBoxAnalogInputType-response>) istream)
  "Deserializes a message object of type '<SetCtrlBoxAnalogInputType-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCtrlBoxAnalogInputType-response>)))
  "Returns string type for a service object of type '<SetCtrlBoxAnalogInputType-response>"
  "dsr_msgs/SetCtrlBoxAnalogInputTypeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCtrlBoxAnalogInputType-response)))
  "Returns string type for a service object of type 'SetCtrlBoxAnalogInputType-response"
  "dsr_msgs/SetCtrlBoxAnalogInputTypeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCtrlBoxAnalogInputType-response>)))
  "Returns md5sum for a message object of type '<SetCtrlBoxAnalogInputType-response>"
  "edb11f75523beab953b2183675ee124d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCtrlBoxAnalogInputType-response)))
  "Returns md5sum for a message object of type 'SetCtrlBoxAnalogInputType-response"
  "edb11f75523beab953b2183675ee124d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCtrlBoxAnalogInputType-response>)))
  "Returns full string definition for message of type '<SetCtrlBoxAnalogInputType-response>"
  (cl:format cl:nil "bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCtrlBoxAnalogInputType-response)))
  "Returns full string definition for message of type 'SetCtrlBoxAnalogInputType-response"
  (cl:format cl:nil "bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCtrlBoxAnalogInputType-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCtrlBoxAnalogInputType-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCtrlBoxAnalogInputType-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetCtrlBoxAnalogInputType)))
  'SetCtrlBoxAnalogInputType-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetCtrlBoxAnalogInputType)))
  'SetCtrlBoxAnalogInputType-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCtrlBoxAnalogInputType)))
  "Returns string type for a service object of type '<SetCtrlBoxAnalogInputType>"
  "dsr_msgs/SetCtrlBoxAnalogInputType")