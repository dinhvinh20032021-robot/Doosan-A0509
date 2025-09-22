; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetRobotSpeedMode-request.msg.html

(cl:defclass <SetRobotSpeedMode-request> (roslisp-msg-protocol:ros-message)
  ((speed_mode
    :reader speed_mode
    :initarg :speed_mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetRobotSpeedMode-request (<SetRobotSpeedMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRobotSpeedMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRobotSpeedMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetRobotSpeedMode-request> is deprecated: use dsr_msgs-srv:SetRobotSpeedMode-request instead.")))

(cl:ensure-generic-function 'speed_mode-val :lambda-list '(m))
(cl:defmethod speed_mode-val ((m <SetRobotSpeedMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:speed_mode-val is deprecated.  Use dsr_msgs-srv:speed_mode instead.")
  (speed_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRobotSpeedMode-request>) ostream)
  "Serializes a message object of type '<SetRobotSpeedMode-request>"
  (cl:let* ((signed (cl:slot-value msg 'speed_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRobotSpeedMode-request>) istream)
  "Deserializes a message object of type '<SetRobotSpeedMode-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRobotSpeedMode-request>)))
  "Returns string type for a service object of type '<SetRobotSpeedMode-request>"
  "dsr_msgs/SetRobotSpeedModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotSpeedMode-request)))
  "Returns string type for a service object of type 'SetRobotSpeedMode-request"
  "dsr_msgs/SetRobotSpeedModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRobotSpeedMode-request>)))
  "Returns md5sum for a message object of type '<SetRobotSpeedMode-request>"
  "724227601ad811bee3732775fc3c79d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRobotSpeedMode-request)))
  "Returns md5sum for a message object of type 'SetRobotSpeedMode-request"
  "724227601ad811bee3732775fc3c79d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRobotSpeedMode-request>)))
  "Returns full string definition for message of type '<SetRobotSpeedMode-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_robot_speed_mode~%#____________________________________________________________________________________________~%~%int8 speed_mode # 0 : SPEED_NORMAL_MODE, 1 : SPEED_REDUCED_MODE~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRobotSpeedMode-request)))
  "Returns full string definition for message of type 'SetRobotSpeedMode-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_robot_speed_mode~%#____________________________________________________________________________________________~%~%int8 speed_mode # 0 : SPEED_NORMAL_MODE, 1 : SPEED_REDUCED_MODE~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRobotSpeedMode-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRobotSpeedMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRobotSpeedMode-request
    (cl:cons ':speed_mode (speed_mode msg))
))
;//! \htmlinclude SetRobotSpeedMode-response.msg.html

(cl:defclass <SetRobotSpeedMode-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetRobotSpeedMode-response (<SetRobotSpeedMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRobotSpeedMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRobotSpeedMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetRobotSpeedMode-response> is deprecated: use dsr_msgs-srv:SetRobotSpeedMode-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetRobotSpeedMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRobotSpeedMode-response>) ostream)
  "Serializes a message object of type '<SetRobotSpeedMode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRobotSpeedMode-response>) istream)
  "Deserializes a message object of type '<SetRobotSpeedMode-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRobotSpeedMode-response>)))
  "Returns string type for a service object of type '<SetRobotSpeedMode-response>"
  "dsr_msgs/SetRobotSpeedModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotSpeedMode-response)))
  "Returns string type for a service object of type 'SetRobotSpeedMode-response"
  "dsr_msgs/SetRobotSpeedModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRobotSpeedMode-response>)))
  "Returns md5sum for a message object of type '<SetRobotSpeedMode-response>"
  "724227601ad811bee3732775fc3c79d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRobotSpeedMode-response)))
  "Returns md5sum for a message object of type 'SetRobotSpeedMode-response"
  "724227601ad811bee3732775fc3c79d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRobotSpeedMode-response>)))
  "Returns full string definition for message of type '<SetRobotSpeedMode-response>"
  (cl:format cl:nil "bool success    ~%                   ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRobotSpeedMode-response)))
  "Returns full string definition for message of type 'SetRobotSpeedMode-response"
  (cl:format cl:nil "bool success    ~%                   ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRobotSpeedMode-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRobotSpeedMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRobotSpeedMode-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetRobotSpeedMode)))
  'SetRobotSpeedMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetRobotSpeedMode)))
  'SetRobotSpeedMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotSpeedMode)))
  "Returns string type for a service object of type '<SetRobotSpeedMode>"
  "dsr_msgs/SetRobotSpeedMode")