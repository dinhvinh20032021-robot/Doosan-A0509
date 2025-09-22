; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetDrlState-request.msg.html

(cl:defclass <GetDrlState-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetDrlState-request (<GetDrlState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDrlState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDrlState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetDrlState-request> is deprecated: use dsr_msgs-srv:GetDrlState-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDrlState-request>) ostream)
  "Serializes a message object of type '<GetDrlState-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDrlState-request>) istream)
  "Deserializes a message object of type '<GetDrlState-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDrlState-request>)))
  "Returns string type for a service object of type '<GetDrlState-request>"
  "dsr_msgs/GetDrlStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDrlState-request)))
  "Returns string type for a service object of type 'GetDrlState-request"
  "dsr_msgs/GetDrlStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDrlState-request>)))
  "Returns md5sum for a message object of type '<GetDrlState-request>"
  "9dd00c5d3b26eace3288075b31af3114")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDrlState-request)))
  "Returns md5sum for a message object of type 'GetDrlState-request"
  "9dd00c5d3b26eace3288075b31af3114")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDrlState-request>)))
  "Returns full string definition for message of type '<GetDrlState-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_drl_state~%# Get DRL Program State~%# 0 : DRL_PROGRAM_STATE_PLAY~%# 1 : DRL_PROGRAM_STATE_STOP~%# 2 : DRL_PROGRAM_STATE_HOLD~%# 3 : DRL_PROGRAM_STATE_LAST~%# drfl.GetProgramState()~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDrlState-request)))
  "Returns full string definition for message of type 'GetDrlState-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_drl_state~%# Get DRL Program State~%# 0 : DRL_PROGRAM_STATE_PLAY~%# 1 : DRL_PROGRAM_STATE_STOP~%# 2 : DRL_PROGRAM_STATE_HOLD~%# 3 : DRL_PROGRAM_STATE_LAST~%# drfl.GetProgramState()~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDrlState-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDrlState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDrlState-request
))
;//! \htmlinclude GetDrlState-response.msg.html

(cl:defclass <GetDrlState-response> (roslisp-msg-protocol:ros-message)
  ((drl_state
    :reader drl_state
    :initarg :drl_state
    :type cl:fixnum
    :initform 0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetDrlState-response (<GetDrlState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDrlState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDrlState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetDrlState-response> is deprecated: use dsr_msgs-srv:GetDrlState-response instead.")))

(cl:ensure-generic-function 'drl_state-val :lambda-list '(m))
(cl:defmethod drl_state-val ((m <GetDrlState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:drl_state-val is deprecated.  Use dsr_msgs-srv:drl_state instead.")
  (drl_state m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetDrlState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDrlState-response>) ostream)
  "Serializes a message object of type '<GetDrlState-response>"
  (cl:let* ((signed (cl:slot-value msg 'drl_state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDrlState-response>) istream)
  "Deserializes a message object of type '<GetDrlState-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drl_state) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDrlState-response>)))
  "Returns string type for a service object of type '<GetDrlState-response>"
  "dsr_msgs/GetDrlStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDrlState-response)))
  "Returns string type for a service object of type 'GetDrlState-response"
  "dsr_msgs/GetDrlStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDrlState-response>)))
  "Returns md5sum for a message object of type '<GetDrlState-response>"
  "9dd00c5d3b26eace3288075b31af3114")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDrlState-response)))
  "Returns md5sum for a message object of type 'GetDrlState-response"
  "9dd00c5d3b26eace3288075b31af3114")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDrlState-response>)))
  "Returns full string definition for message of type '<GetDrlState-response>"
  (cl:format cl:nil "int8        drl_state # <DRL_PROGRAM_STATE>~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDrlState-response)))
  "Returns full string definition for message of type 'GetDrlState-response"
  (cl:format cl:nil "int8        drl_state # <DRL_PROGRAM_STATE>~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDrlState-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDrlState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDrlState-response
    (cl:cons ':drl_state (drl_state msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetDrlState)))
  'GetDrlState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetDrlState)))
  'GetDrlState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDrlState)))
  "Returns string type for a service object of type '<GetDrlState>"
  "dsr_msgs/GetDrlState")