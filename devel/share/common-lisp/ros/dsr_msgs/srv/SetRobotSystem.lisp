; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetRobotSystem-request.msg.html

(cl:defclass <SetRobotSystem-request> (roslisp-msg-protocol:ros-message)
  ((robot_system
    :reader robot_system
    :initarg :robot_system
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetRobotSystem-request (<SetRobotSystem-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRobotSystem-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRobotSystem-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetRobotSystem-request> is deprecated: use dsr_msgs-srv:SetRobotSystem-request instead.")))

(cl:ensure-generic-function 'robot_system-val :lambda-list '(m))
(cl:defmethod robot_system-val ((m <SetRobotSystem-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:robot_system-val is deprecated.  Use dsr_msgs-srv:robot_system instead.")
  (robot_system m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRobotSystem-request>) ostream)
  "Serializes a message object of type '<SetRobotSystem-request>"
  (cl:let* ((signed (cl:slot-value msg 'robot_system)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRobotSystem-request>) istream)
  "Deserializes a message object of type '<SetRobotSystem-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_system) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRobotSystem-request>)))
  "Returns string type for a service object of type '<SetRobotSystem-request>"
  "dsr_msgs/SetRobotSystemRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotSystem-request)))
  "Returns string type for a service object of type 'SetRobotSystem-request"
  "dsr_msgs/SetRobotSystemRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRobotSystem-request>)))
  "Returns md5sum for a message object of type '<SetRobotSystem-request>"
  "5474fc13fdf01710b0f371e4b3b11c86")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRobotSystem-request)))
  "Returns md5sum for a message object of type 'SetRobotSystem-request"
  "5474fc13fdf01710b0f371e4b3b11c86")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRobotSystem-request>)))
  "Returns full string definition for message of type '<SetRobotSystem-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_robot_system~%#____________________________________________________________________________________________~%~%int8 robot_system   # 0 : ROBOT_SYSTEM_REAL, 1 : ROBOT_SYSTEM_VIRTUAL~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRobotSystem-request)))
  "Returns full string definition for message of type 'SetRobotSystem-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_robot_system~%#____________________________________________________________________________________________~%~%int8 robot_system   # 0 : ROBOT_SYSTEM_REAL, 1 : ROBOT_SYSTEM_VIRTUAL~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRobotSystem-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRobotSystem-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRobotSystem-request
    (cl:cons ':robot_system (robot_system msg))
))
;//! \htmlinclude SetRobotSystem-response.msg.html

(cl:defclass <SetRobotSystem-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetRobotSystem-response (<SetRobotSystem-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRobotSystem-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRobotSystem-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetRobotSystem-response> is deprecated: use dsr_msgs-srv:SetRobotSystem-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetRobotSystem-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRobotSystem-response>) ostream)
  "Serializes a message object of type '<SetRobotSystem-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRobotSystem-response>) istream)
  "Deserializes a message object of type '<SetRobotSystem-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRobotSystem-response>)))
  "Returns string type for a service object of type '<SetRobotSystem-response>"
  "dsr_msgs/SetRobotSystemResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotSystem-response)))
  "Returns string type for a service object of type 'SetRobotSystem-response"
  "dsr_msgs/SetRobotSystemResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRobotSystem-response>)))
  "Returns md5sum for a message object of type '<SetRobotSystem-response>"
  "5474fc13fdf01710b0f371e4b3b11c86")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRobotSystem-response)))
  "Returns md5sum for a message object of type 'SetRobotSystem-response"
  "5474fc13fdf01710b0f371e4b3b11c86")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRobotSystem-response>)))
  "Returns full string definition for message of type '<SetRobotSystem-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRobotSystem-response)))
  "Returns full string definition for message of type 'SetRobotSystem-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRobotSystem-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRobotSystem-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRobotSystem-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetRobotSystem)))
  'SetRobotSystem-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetRobotSystem)))
  'SetRobotSystem-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotSystem)))
  "Returns string type for a service object of type '<SetRobotSystem>"
  "dsr_msgs/SetRobotSystem")