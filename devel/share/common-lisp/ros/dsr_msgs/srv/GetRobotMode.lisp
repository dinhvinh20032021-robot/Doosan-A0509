; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetRobotMode-request.msg.html

(cl:defclass <GetRobotMode-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetRobotMode-request (<GetRobotMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetRobotMode-request> is deprecated: use dsr_msgs-srv:GetRobotMode-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotMode-request>) ostream)
  "Serializes a message object of type '<GetRobotMode-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotMode-request>) istream)
  "Deserializes a message object of type '<GetRobotMode-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotMode-request>)))
  "Returns string type for a service object of type '<GetRobotMode-request>"
  "dsr_msgs/GetRobotModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotMode-request)))
  "Returns string type for a service object of type 'GetRobotMode-request"
  "dsr_msgs/GetRobotModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotMode-request>)))
  "Returns md5sum for a message object of type '<GetRobotMode-request>"
  "020eb3bd52e8053b30ef11386e0bba83")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotMode-request)))
  "Returns md5sum for a message object of type 'GetRobotMode-request"
  "020eb3bd52e8053b30ef11386e0bba83")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotMode-request>)))
  "Returns full string definition for message of type '<GetRobotMode-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_robot_mode~%# Return to current robot-mode~%# 0 : ROBOT_MODE_MANUAL~%# 1 : ROBOT_MODE_AUTONOMOUS~%# 2 : ROBOT_MODE_MEASURE~%# drfl.GetRobotMode()~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotMode-request)))
  "Returns full string definition for message of type 'GetRobotMode-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_robot_mode~%# Return to current robot-mode~%# 0 : ROBOT_MODE_MANUAL~%# 1 : ROBOT_MODE_AUTONOMOUS~%# 2 : ROBOT_MODE_MEASURE~%# drfl.GetRobotMode()~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotMode-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotMode-request
))
;//! \htmlinclude GetRobotMode-response.msg.html

(cl:defclass <GetRobotMode-response> (roslisp-msg-protocol:ros-message)
  ((robot_mode
    :reader robot_mode
    :initarg :robot_mode
    :type cl:fixnum
    :initform 0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetRobotMode-response (<GetRobotMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetRobotMode-response> is deprecated: use dsr_msgs-srv:GetRobotMode-response instead.")))

(cl:ensure-generic-function 'robot_mode-val :lambda-list '(m))
(cl:defmethod robot_mode-val ((m <GetRobotMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:robot_mode-val is deprecated.  Use dsr_msgs-srv:robot_mode instead.")
  (robot_mode m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetRobotMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotMode-response>) ostream)
  "Serializes a message object of type '<GetRobotMode-response>"
  (cl:let* ((signed (cl:slot-value msg 'robot_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotMode-response>) istream)
  "Deserializes a message object of type '<GetRobotMode-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotMode-response>)))
  "Returns string type for a service object of type '<GetRobotMode-response>"
  "dsr_msgs/GetRobotModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotMode-response)))
  "Returns string type for a service object of type 'GetRobotMode-response"
  "dsr_msgs/GetRobotModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotMode-response>)))
  "Returns md5sum for a message object of type '<GetRobotMode-response>"
  "020eb3bd52e8053b30ef11386e0bba83")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotMode-response)))
  "Returns md5sum for a message object of type 'GetRobotMode-response"
  "020eb3bd52e8053b30ef11386e0bba83")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotMode-response>)))
  "Returns full string definition for message of type '<GetRobotMode-response>"
  (cl:format cl:nil "int8 robot_mode~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotMode-response)))
  "Returns full string definition for message of type 'GetRobotMode-response"
  (cl:format cl:nil "int8 robot_mode~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotMode-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotMode-response
    (cl:cons ':robot_mode (robot_mode msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetRobotMode)))
  'GetRobotMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetRobotMode)))
  'GetRobotMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotMode)))
  "Returns string type for a service object of type '<GetRobotMode>"
  "dsr_msgs/GetRobotMode")