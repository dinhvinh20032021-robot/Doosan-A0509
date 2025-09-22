; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetRobotMode-request.msg.html

(cl:defclass <SetRobotMode-request> (roslisp-msg-protocol:ros-message)
  ((robot_mode
    :reader robot_mode
    :initarg :robot_mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetRobotMode-request (<SetRobotMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRobotMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRobotMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetRobotMode-request> is deprecated: use dsr_msgs-srv:SetRobotMode-request instead.")))

(cl:ensure-generic-function 'robot_mode-val :lambda-list '(m))
(cl:defmethod robot_mode-val ((m <SetRobotMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:robot_mode-val is deprecated.  Use dsr_msgs-srv:robot_mode instead.")
  (robot_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRobotMode-request>) ostream)
  "Serializes a message object of type '<SetRobotMode-request>"
  (cl:let* ((signed (cl:slot-value msg 'robot_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRobotMode-request>) istream)
  "Deserializes a message object of type '<SetRobotMode-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRobotMode-request>)))
  "Returns string type for a service object of type '<SetRobotMode-request>"
  "dsr_msgs/SetRobotModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotMode-request)))
  "Returns string type for a service object of type 'SetRobotMode-request"
  "dsr_msgs/SetRobotModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRobotMode-request>)))
  "Returns md5sum for a message object of type '<SetRobotMode-request>"
  "1839172555f03e6135521c0cb3b452f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRobotMode-request)))
  "Returns md5sum for a message object of type 'SetRobotMode-request"
  "1839172555f03e6135521c0cb3b452f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRobotMode-request>)))
  "Returns full string definition for message of type '<SetRobotMode-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_robot_mode~%# Change the robot-mode~%# 0 : ROBOT_MODE_MANUAL~%# 1 : ROBOT_MODE_AUTONOMOUS~%# 2 :ROBOT_MODE_MEASURE~%# drfl.SetRobotMode()~%#____________________________________________________________________________________________~%~%int8 robot_mode # <Robot_Mode>~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRobotMode-request)))
  "Returns full string definition for message of type 'SetRobotMode-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_robot_mode~%# Change the robot-mode~%# 0 : ROBOT_MODE_MANUAL~%# 1 : ROBOT_MODE_AUTONOMOUS~%# 2 :ROBOT_MODE_MEASURE~%# drfl.SetRobotMode()~%#____________________________________________________________________________________________~%~%int8 robot_mode # <Robot_Mode>~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRobotMode-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRobotMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRobotMode-request
    (cl:cons ':robot_mode (robot_mode msg))
))
;//! \htmlinclude SetRobotMode-response.msg.html

(cl:defclass <SetRobotMode-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetRobotMode-response (<SetRobotMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRobotMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRobotMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetRobotMode-response> is deprecated: use dsr_msgs-srv:SetRobotMode-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetRobotMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRobotMode-response>) ostream)
  "Serializes a message object of type '<SetRobotMode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRobotMode-response>) istream)
  "Deserializes a message object of type '<SetRobotMode-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRobotMode-response>)))
  "Returns string type for a service object of type '<SetRobotMode-response>"
  "dsr_msgs/SetRobotModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotMode-response)))
  "Returns string type for a service object of type 'SetRobotMode-response"
  "dsr_msgs/SetRobotModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRobotMode-response>)))
  "Returns md5sum for a message object of type '<SetRobotMode-response>"
  "1839172555f03e6135521c0cb3b452f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRobotMode-response)))
  "Returns md5sum for a message object of type 'SetRobotMode-response"
  "1839172555f03e6135521c0cb3b452f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRobotMode-response>)))
  "Returns full string definition for message of type '<SetRobotMode-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRobotMode-response)))
  "Returns full string definition for message of type 'SetRobotMode-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRobotMode-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRobotMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRobotMode-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetRobotMode)))
  'SetRobotMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetRobotMode)))
  'SetRobotMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotMode)))
  "Returns string type for a service object of type '<SetRobotMode>"
  "dsr_msgs/SetRobotMode")