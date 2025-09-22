; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-msg)


;//! \htmlinclude RobotStop.msg.html

(cl:defclass <RobotStop> (roslisp-msg-protocol:ros-message)
  ((stop_mode
    :reader stop_mode
    :initarg :stop_mode
    :type cl:integer
    :initform 0))
)

(cl:defclass RobotStop (<RobotStop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotStop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotStop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-msg:<RobotStop> is deprecated: use dsr_msgs-msg:RobotStop instead.")))

(cl:ensure-generic-function 'stop_mode-val :lambda-list '(m))
(cl:defmethod stop_mode-val ((m <RobotStop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:stop_mode-val is deprecated.  Use dsr_msgs-msg:stop_mode instead.")
  (stop_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotStop>) ostream)
  "Serializes a message object of type '<RobotStop>"
  (cl:let* ((signed (cl:slot-value msg 'stop_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotStop>) istream)
  "Deserializes a message object of type '<RobotStop>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stop_mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotStop>)))
  "Returns string type for a message object of type '<RobotStop>"
  "dsr_msgs/RobotStop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotStop)))
  "Returns string type for a message object of type 'RobotStop"
  "dsr_msgs/RobotStop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotStop>)))
  "Returns md5sum for a message object of type '<RobotStop>"
  "82712390efeed0d0668a551e004b332c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotStop)))
  "Returns md5sum for a message object of type 'RobotStop"
  "82712390efeed0d0668a551e004b332c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotStop>)))
  "Returns full string definition for message of type '<RobotStop>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# [ robot stop mode ] ~%# 0 : STOP_TYPE_QUICK_STO~%# 1 : STOP_TYPE_QUICK~%# 2 : STOP_TYPE_SLOW~%# 3 : STOP_TYPE_HOLD = STOP_TYPE_EMERGENCY~%#~%#____________________________________________________________________________________________~%~%int32 stop_mode  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotStop)))
  "Returns full string definition for message of type 'RobotStop"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# [ robot stop mode ] ~%# 0 : STOP_TYPE_QUICK_STO~%# 1 : STOP_TYPE_QUICK~%# 2 : STOP_TYPE_SLOW~%# 3 : STOP_TYPE_HOLD = STOP_TYPE_EMERGENCY~%#~%#____________________________________________________________________________________________~%~%int32 stop_mode  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotStop>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotStop>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotStop
    (cl:cons ':stop_mode (stop_mode msg))
))
