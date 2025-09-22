; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude MoveStop-request.msg.html

(cl:defclass <MoveStop-request> (roslisp-msg-protocol:ros-message)
  ((stop_mode
    :reader stop_mode
    :initarg :stop_mode
    :type cl:integer
    :initform 0))
)

(cl:defclass MoveStop-request (<MoveStop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveStop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveStop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<MoveStop-request> is deprecated: use dsr_msgs-srv:MoveStop-request instead.")))

(cl:ensure-generic-function 'stop_mode-val :lambda-list '(m))
(cl:defmethod stop_mode-val ((m <MoveStop-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:stop_mode-val is deprecated.  Use dsr_msgs-srv:stop_mode instead.")
  (stop_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveStop-request>) ostream)
  "Serializes a message object of type '<MoveStop-request>"
  (cl:let* ((signed (cl:slot-value msg 'stop_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveStop-request>) istream)
  "Deserializes a message object of type '<MoveStop-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stop_mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveStop-request>)))
  "Returns string type for a service object of type '<MoveStop-request>"
  "dsr_msgs/MoveStopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveStop-request)))
  "Returns string type for a service object of type 'MoveStop-request"
  "dsr_msgs/MoveStopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveStop-request>)))
  "Returns md5sum for a message object of type '<MoveStop-request>"
  "a0a25e9e76d4431108e6e2682aa29331")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveStop-request)))
  "Returns md5sum for a message object of type 'MoveStop-request"
  "a0a25e9e76d4431108e6e2682aa29331")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveStop-request>)))
  "Returns full string definition for message of type '<MoveStop-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# stop()~%# 인자 설명 추가 필요!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!~%#____________________________________________________________________________________________~%~%int32 stop_mode         # DR_QSTOP_STO(0) : Quick stop (Stop Category 1 without STO(Safe Torque Off)~%                        # DR_QSTOP(1)     : Quick stop (Stop Category 2)~%                        # DR_SSTO(2)      : Soft Stop~%                        # DR_HOLD(3)      : HOLD stop~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveStop-request)))
  "Returns full string definition for message of type 'MoveStop-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# stop()~%# 인자 설명 추가 필요!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!~%#____________________________________________________________________________________________~%~%int32 stop_mode         # DR_QSTOP_STO(0) : Quick stop (Stop Category 1 without STO(Safe Torque Off)~%                        # DR_QSTOP(1)     : Quick stop (Stop Category 2)~%                        # DR_SSTO(2)      : Soft Stop~%                        # DR_HOLD(3)      : HOLD stop~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveStop-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveStop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveStop-request
    (cl:cons ':stop_mode (stop_mode msg))
))
;//! \htmlinclude MoveStop-response.msg.html

(cl:defclass <MoveStop-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoveStop-response (<MoveStop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveStop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveStop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<MoveStop-response> is deprecated: use dsr_msgs-srv:MoveStop-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <MoveStop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveStop-response>) ostream)
  "Serializes a message object of type '<MoveStop-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveStop-response>) istream)
  "Deserializes a message object of type '<MoveStop-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveStop-response>)))
  "Returns string type for a service object of type '<MoveStop-response>"
  "dsr_msgs/MoveStopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveStop-response)))
  "Returns string type for a service object of type 'MoveStop-response"
  "dsr_msgs/MoveStopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveStop-response>)))
  "Returns md5sum for a message object of type '<MoveStop-response>"
  "a0a25e9e76d4431108e6e2682aa29331")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveStop-response)))
  "Returns md5sum for a message object of type 'MoveStop-response"
  "a0a25e9e76d4431108e6e2682aa29331")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveStop-response>)))
  "Returns full string definition for message of type '<MoveStop-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveStop-response)))
  "Returns full string definition for message of type 'MoveStop-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveStop-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveStop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveStop-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveStop)))
  'MoveStop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveStop)))
  'MoveStop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveStop)))
  "Returns string type for a service object of type '<MoveStop>"
  "dsr_msgs/MoveStop")