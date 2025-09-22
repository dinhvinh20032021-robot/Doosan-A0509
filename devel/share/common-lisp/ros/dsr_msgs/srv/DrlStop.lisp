; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude DrlStop-request.msg.html

(cl:defclass <DrlStop-request> (roslisp-msg-protocol:ros-message)
  ((stop_mode
    :reader stop_mode
    :initarg :stop_mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass DrlStop-request (<DrlStop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DrlStop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DrlStop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<DrlStop-request> is deprecated: use dsr_msgs-srv:DrlStop-request instead.")))

(cl:ensure-generic-function 'stop_mode-val :lambda-list '(m))
(cl:defmethod stop_mode-val ((m <DrlStop-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:stop_mode-val is deprecated.  Use dsr_msgs-srv:stop_mode instead.")
  (stop_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DrlStop-request>) ostream)
  "Serializes a message object of type '<DrlStop-request>"
  (cl:let* ((signed (cl:slot-value msg 'stop_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DrlStop-request>) istream)
  "Deserializes a message object of type '<DrlStop-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stop_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DrlStop-request>)))
  "Returns string type for a service object of type '<DrlStop-request>"
  "dsr_msgs/DrlStopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrlStop-request)))
  "Returns string type for a service object of type 'DrlStop-request"
  "dsr_msgs/DrlStopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DrlStop-request>)))
  "Returns md5sum for a message object of type '<DrlStop-request>"
  "a7f8fadac11dc7732e1bdb5e5d128820")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DrlStop-request)))
  "Returns md5sum for a message object of type 'DrlStop-request"
  "a7f8fadac11dc7732e1bdb5e5d128820")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DrlStop-request>)))
  "Returns full string definition for message of type '<DrlStop-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# drl_script_stop  ~%# STOP_TYPE_QUICK_STO = 0~%# STOP_TYPE_QUICK     = 1~%# STOP_TYPE_SLOW      = 2~%# STOP_TYPE_HOLD = STOP_TYPE_EMERGENCY = 3  ~%#____________________________________________________________________________________________~%~%int8    stop_mode       # <STOP_TYPE> stop_mode       ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DrlStop-request)))
  "Returns full string definition for message of type 'DrlStop-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# drl_script_stop  ~%# STOP_TYPE_QUICK_STO = 0~%# STOP_TYPE_QUICK     = 1~%# STOP_TYPE_SLOW      = 2~%# STOP_TYPE_HOLD = STOP_TYPE_EMERGENCY = 3  ~%#____________________________________________________________________________________________~%~%int8    stop_mode       # <STOP_TYPE> stop_mode       ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DrlStop-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DrlStop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DrlStop-request
    (cl:cons ':stop_mode (stop_mode msg))
))
;//! \htmlinclude DrlStop-response.msg.html

(cl:defclass <DrlStop-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DrlStop-response (<DrlStop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DrlStop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DrlStop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<DrlStop-response> is deprecated: use dsr_msgs-srv:DrlStop-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DrlStop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DrlStop-response>) ostream)
  "Serializes a message object of type '<DrlStop-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DrlStop-response>) istream)
  "Deserializes a message object of type '<DrlStop-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DrlStop-response>)))
  "Returns string type for a service object of type '<DrlStop-response>"
  "dsr_msgs/DrlStopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrlStop-response)))
  "Returns string type for a service object of type 'DrlStop-response"
  "dsr_msgs/DrlStopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DrlStop-response>)))
  "Returns md5sum for a message object of type '<DrlStop-response>"
  "a7f8fadac11dc7732e1bdb5e5d128820")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DrlStop-response)))
  "Returns md5sum for a message object of type 'DrlStop-response"
  "a7f8fadac11dc7732e1bdb5e5d128820")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DrlStop-response>)))
  "Returns full string definition for message of type '<DrlStop-response>"
  (cl:format cl:nil "bool    success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DrlStop-response)))
  "Returns full string definition for message of type 'DrlStop-response"
  (cl:format cl:nil "bool    success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DrlStop-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DrlStop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DrlStop-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DrlStop)))
  'DrlStop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DrlStop)))
  'DrlStop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrlStop)))
  "Returns string type for a service object of type '<DrlStop>"
  "dsr_msgs/DrlStop")