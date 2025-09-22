; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetRobotControl-request.msg.html

(cl:defclass <SetRobotControl-request> (roslisp-msg-protocol:ros-message)
  ((robot_control
    :reader robot_control
    :initarg :robot_control
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetRobotControl-request (<SetRobotControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRobotControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRobotControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetRobotControl-request> is deprecated: use dsr_msgs-srv:SetRobotControl-request instead.")))

(cl:ensure-generic-function 'robot_control-val :lambda-list '(m))
(cl:defmethod robot_control-val ((m <SetRobotControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:robot_control-val is deprecated.  Use dsr_msgs-srv:robot_control instead.")
  (robot_control m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRobotControl-request>) ostream)
  "Serializes a message object of type '<SetRobotControl-request>"
  (cl:let* ((signed (cl:slot-value msg 'robot_control)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRobotControl-request>) istream)
  "Deserializes a message object of type '<SetRobotControl-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_control) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRobotControl-request>)))
  "Returns string type for a service object of type '<SetRobotControl-request>"
  "dsr_msgs/SetRobotControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotControl-request)))
  "Returns string type for a service object of type 'SetRobotControl-request"
  "dsr_msgs/SetRobotControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRobotControl-request>)))
  "Returns md5sum for a message object of type '<SetRobotControl-request>"
  "e3c7c3cfa0120a8cf83abd30ec7bcbc3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRobotControl-request)))
  "Returns md5sum for a message object of type 'SetRobotControl-request"
  "e3c7c3cfa0120a8cf83abd30ec7bcbc3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRobotControl-request>)))
  "Returns full string definition for message of type '<SetRobotControl-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_robot_control~%#____________________________________________________________________________________________~%~%int8 robot_control # 0 : CONTROL_INIT_CONFIG~%                   # 1 : CONTROL_ENABLE_OPERATION~%                   # 2 : CONTROL_RESET_SAFET_STOP~%                   # 3 : CONTROL_RESET_SAFE_STOP = CONTROL_RESET_SAFET_STOP,~%                   # 4 : CONTROL_RESET_SAFET_OFF,~%                   # 5 : CONTROL_RESET_SAFE_OFF = CONTROL_RESET_SAFET_OFF,~%                   # 6 : CONTROL_SERVO_ON = CONTROL_RESET_SAFET_OFF,~%                   # 7 : CONTROL_RECOVERY_SAFE_STOP,~%                   # 8 : CONTROL_RECOVERY_SAFE_OFF,~%                   # 9 : CONTROL_RECOVERY_BACKDRIVE,~%                   # 10: CONTROL_RESET_RECOVERY,~%                   # 11: CONTROL_LAST~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRobotControl-request)))
  "Returns full string definition for message of type 'SetRobotControl-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_robot_control~%#____________________________________________________________________________________________~%~%int8 robot_control # 0 : CONTROL_INIT_CONFIG~%                   # 1 : CONTROL_ENABLE_OPERATION~%                   # 2 : CONTROL_RESET_SAFET_STOP~%                   # 3 : CONTROL_RESET_SAFE_STOP = CONTROL_RESET_SAFET_STOP,~%                   # 4 : CONTROL_RESET_SAFET_OFF,~%                   # 5 : CONTROL_RESET_SAFE_OFF = CONTROL_RESET_SAFET_OFF,~%                   # 6 : CONTROL_SERVO_ON = CONTROL_RESET_SAFET_OFF,~%                   # 7 : CONTROL_RECOVERY_SAFE_STOP,~%                   # 8 : CONTROL_RECOVERY_SAFE_OFF,~%                   # 9 : CONTROL_RECOVERY_BACKDRIVE,~%                   # 10: CONTROL_RESET_RECOVERY,~%                   # 11: CONTROL_LAST~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRobotControl-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRobotControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRobotControl-request
    (cl:cons ':robot_control (robot_control msg))
))
;//! \htmlinclude SetRobotControl-response.msg.html

(cl:defclass <SetRobotControl-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetRobotControl-response (<SetRobotControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRobotControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRobotControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetRobotControl-response> is deprecated: use dsr_msgs-srv:SetRobotControl-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetRobotControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRobotControl-response>) ostream)
  "Serializes a message object of type '<SetRobotControl-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRobotControl-response>) istream)
  "Deserializes a message object of type '<SetRobotControl-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRobotControl-response>)))
  "Returns string type for a service object of type '<SetRobotControl-response>"
  "dsr_msgs/SetRobotControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotControl-response)))
  "Returns string type for a service object of type 'SetRobotControl-response"
  "dsr_msgs/SetRobotControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRobotControl-response>)))
  "Returns md5sum for a message object of type '<SetRobotControl-response>"
  "e3c7c3cfa0120a8cf83abd30ec7bcbc3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRobotControl-response)))
  "Returns md5sum for a message object of type 'SetRobotControl-response"
  "e3c7c3cfa0120a8cf83abd30ec7bcbc3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRobotControl-response>)))
  "Returns full string definition for message of type '<SetRobotControl-response>"
  (cl:format cl:nil "bool success    ~%                   ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRobotControl-response)))
  "Returns full string definition for message of type 'SetRobotControl-response"
  (cl:format cl:nil "bool success    ~%                   ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRobotControl-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRobotControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRobotControl-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetRobotControl)))
  'SetRobotControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetRobotControl)))
  'SetRobotControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotControl)))
  "Returns string type for a service object of type '<SetRobotControl>"
  "dsr_msgs/SetRobotControl")