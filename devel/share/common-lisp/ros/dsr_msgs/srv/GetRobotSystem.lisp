; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetRobotSystem-request.msg.html

(cl:defclass <GetRobotSystem-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetRobotSystem-request (<GetRobotSystem-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotSystem-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotSystem-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetRobotSystem-request> is deprecated: use dsr_msgs-srv:GetRobotSystem-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotSystem-request>) ostream)
  "Serializes a message object of type '<GetRobotSystem-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotSystem-request>) istream)
  "Deserializes a message object of type '<GetRobotSystem-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotSystem-request>)))
  "Returns string type for a service object of type '<GetRobotSystem-request>"
  "dsr_msgs/GetRobotSystemRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotSystem-request)))
  "Returns string type for a service object of type 'GetRobotSystem-request"
  "dsr_msgs/GetRobotSystemRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotSystem-request>)))
  "Returns md5sum for a message object of type '<GetRobotSystem-request>"
  "20308dcb70c4e3f1df099ac95a676cf2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotSystem-request)))
  "Returns md5sum for a message object of type 'GetRobotSystem-request"
  "20308dcb70c4e3f1df099ac95a676cf2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotSystem-request>)))
  "Returns full string definition for message of type '<GetRobotSystem-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_robot_system~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotSystem-request)))
  "Returns full string definition for message of type 'GetRobotSystem-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_robot_system~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotSystem-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotSystem-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotSystem-request
))
;//! \htmlinclude GetRobotSystem-response.msg.html

(cl:defclass <GetRobotSystem-response> (roslisp-msg-protocol:ros-message)
  ((robot_system
    :reader robot_system
    :initarg :robot_system
    :type cl:fixnum
    :initform 0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetRobotSystem-response (<GetRobotSystem-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotSystem-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotSystem-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetRobotSystem-response> is deprecated: use dsr_msgs-srv:GetRobotSystem-response instead.")))

(cl:ensure-generic-function 'robot_system-val :lambda-list '(m))
(cl:defmethod robot_system-val ((m <GetRobotSystem-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:robot_system-val is deprecated.  Use dsr_msgs-srv:robot_system instead.")
  (robot_system m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetRobotSystem-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotSystem-response>) ostream)
  "Serializes a message object of type '<GetRobotSystem-response>"
  (cl:let* ((signed (cl:slot-value msg 'robot_system)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotSystem-response>) istream)
  "Deserializes a message object of type '<GetRobotSystem-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_system) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotSystem-response>)))
  "Returns string type for a service object of type '<GetRobotSystem-response>"
  "dsr_msgs/GetRobotSystemResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotSystem-response)))
  "Returns string type for a service object of type 'GetRobotSystem-response"
  "dsr_msgs/GetRobotSystemResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotSystem-response>)))
  "Returns md5sum for a message object of type '<GetRobotSystem-response>"
  "20308dcb70c4e3f1df099ac95a676cf2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotSystem-response)))
  "Returns md5sum for a message object of type 'GetRobotSystem-response"
  "20308dcb70c4e3f1df099ac95a676cf2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotSystem-response>)))
  "Returns full string definition for message of type '<GetRobotSystem-response>"
  (cl:format cl:nil "int8 robot_system   # 0 : ROBOT_SYSTEM_REAL~%                    # 1 : ROBOT_SYSTEM_VIRTUAL~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotSystem-response)))
  "Returns full string definition for message of type 'GetRobotSystem-response"
  (cl:format cl:nil "int8 robot_system   # 0 : ROBOT_SYSTEM_REAL~%                    # 1 : ROBOT_SYSTEM_VIRTUAL~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotSystem-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotSystem-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotSystem-response
    (cl:cons ':robot_system (robot_system msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetRobotSystem)))
  'GetRobotSystem-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetRobotSystem)))
  'GetRobotSystem-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotSystem)))
  "Returns string type for a service object of type '<GetRobotSystem>"
  "dsr_msgs/GetRobotSystem")