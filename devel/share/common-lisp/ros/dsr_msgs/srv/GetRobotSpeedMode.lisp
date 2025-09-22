; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetRobotSpeedMode-request.msg.html

(cl:defclass <GetRobotSpeedMode-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetRobotSpeedMode-request (<GetRobotSpeedMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotSpeedMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotSpeedMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetRobotSpeedMode-request> is deprecated: use dsr_msgs-srv:GetRobotSpeedMode-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotSpeedMode-request>) ostream)
  "Serializes a message object of type '<GetRobotSpeedMode-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotSpeedMode-request>) istream)
  "Deserializes a message object of type '<GetRobotSpeedMode-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotSpeedMode-request>)))
  "Returns string type for a service object of type '<GetRobotSpeedMode-request>"
  "dsr_msgs/GetRobotSpeedModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotSpeedMode-request)))
  "Returns string type for a service object of type 'GetRobotSpeedMode-request"
  "dsr_msgs/GetRobotSpeedModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotSpeedMode-request>)))
  "Returns md5sum for a message object of type '<GetRobotSpeedMode-request>"
  "b5d54772deeae3778247a5930ed4c1f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotSpeedMode-request)))
  "Returns md5sum for a message object of type 'GetRobotSpeedMode-request"
  "b5d54772deeae3778247a5930ed4c1f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotSpeedMode-request>)))
  "Returns full string definition for message of type '<GetRobotSpeedMode-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_robot_speed_mode~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotSpeedMode-request)))
  "Returns full string definition for message of type 'GetRobotSpeedMode-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_robot_speed_mode~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotSpeedMode-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotSpeedMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotSpeedMode-request
))
;//! \htmlinclude GetRobotSpeedMode-response.msg.html

(cl:defclass <GetRobotSpeedMode-response> (roslisp-msg-protocol:ros-message)
  ((speed_mode
    :reader speed_mode
    :initarg :speed_mode
    :type cl:fixnum
    :initform 0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetRobotSpeedMode-response (<GetRobotSpeedMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotSpeedMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotSpeedMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetRobotSpeedMode-response> is deprecated: use dsr_msgs-srv:GetRobotSpeedMode-response instead.")))

(cl:ensure-generic-function 'speed_mode-val :lambda-list '(m))
(cl:defmethod speed_mode-val ((m <GetRobotSpeedMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:speed_mode-val is deprecated.  Use dsr_msgs-srv:speed_mode instead.")
  (speed_mode m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetRobotSpeedMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotSpeedMode-response>) ostream)
  "Serializes a message object of type '<GetRobotSpeedMode-response>"
  (cl:let* ((signed (cl:slot-value msg 'speed_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotSpeedMode-response>) istream)
  "Deserializes a message object of type '<GetRobotSpeedMode-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotSpeedMode-response>)))
  "Returns string type for a service object of type '<GetRobotSpeedMode-response>"
  "dsr_msgs/GetRobotSpeedModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotSpeedMode-response)))
  "Returns string type for a service object of type 'GetRobotSpeedMode-response"
  "dsr_msgs/GetRobotSpeedModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotSpeedMode-response>)))
  "Returns md5sum for a message object of type '<GetRobotSpeedMode-response>"
  "b5d54772deeae3778247a5930ed4c1f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotSpeedMode-response)))
  "Returns md5sum for a message object of type 'GetRobotSpeedMode-response"
  "b5d54772deeae3778247a5930ed4c1f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotSpeedMode-response>)))
  "Returns full string definition for message of type '<GetRobotSpeedMode-response>"
  (cl:format cl:nil "int8 speed_mode # 0 : SPEED_NORMAL_MODE~%                # 1 : SPEED_REDUCED_MODE~%bool        success   ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotSpeedMode-response)))
  "Returns full string definition for message of type 'GetRobotSpeedMode-response"
  (cl:format cl:nil "int8 speed_mode # 0 : SPEED_NORMAL_MODE~%                # 1 : SPEED_REDUCED_MODE~%bool        success   ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotSpeedMode-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotSpeedMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotSpeedMode-response
    (cl:cons ':speed_mode (speed_mode msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetRobotSpeedMode)))
  'GetRobotSpeedMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetRobotSpeedMode)))
  'GetRobotSpeedMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotSpeedMode)))
  "Returns string type for a service object of type '<GetRobotSpeedMode>"
  "dsr_msgs/GetRobotSpeedMode")