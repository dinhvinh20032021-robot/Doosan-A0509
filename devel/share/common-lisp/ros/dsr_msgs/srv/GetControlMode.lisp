; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetControlMode-request.msg.html

(cl:defclass <GetControlMode-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetControlMode-request (<GetControlMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControlMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControlMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetControlMode-request> is deprecated: use dsr_msgs-srv:GetControlMode-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControlMode-request>) ostream)
  "Serializes a message object of type '<GetControlMode-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControlMode-request>) istream)
  "Deserializes a message object of type '<GetControlMode-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControlMode-request>)))
  "Returns string type for a service object of type '<GetControlMode-request>"
  "dsr_msgs/GetControlModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControlMode-request)))
  "Returns string type for a service object of type 'GetControlMode-request"
  "dsr_msgs/GetControlModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControlMode-request>)))
  "Returns md5sum for a message object of type '<GetControlMode-request>"
  "bc4e735fc2aafec8676fb72eaa5429bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControlMode-request)))
  "Returns md5sum for a message object of type 'GetControlMode-request"
  "bc4e735fc2aafec8676fb72eaa5429bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControlMode-request>)))
  "Returns full string definition for message of type '<GetControlMode-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_control_mode()  ~%#____________________________________________________________________________________________~%# This service returns the current control mode.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControlMode-request)))
  "Returns full string definition for message of type 'GetControlMode-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_control_mode()  ~%#____________________________________________________________________________________________~%# This service returns the current control mode.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControlMode-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControlMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControlMode-request
))
;//! \htmlinclude GetControlMode-response.msg.html

(cl:defclass <GetControlMode-response> (roslisp-msg-protocol:ros-message)
  ((control_mode
    :reader control_mode
    :initarg :control_mode
    :type cl:fixnum
    :initform 0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetControlMode-response (<GetControlMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControlMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControlMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetControlMode-response> is deprecated: use dsr_msgs-srv:GetControlMode-response instead.")))

(cl:ensure-generic-function 'control_mode-val :lambda-list '(m))
(cl:defmethod control_mode-val ((m <GetControlMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:control_mode-val is deprecated.  Use dsr_msgs-srv:control_mode instead.")
  (control_mode m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetControlMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControlMode-response>) ostream)
  "Serializes a message object of type '<GetControlMode-response>"
  (cl:let* ((signed (cl:slot-value msg 'control_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControlMode-response>) istream)
  "Deserializes a message object of type '<GetControlMode-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'control_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControlMode-response>)))
  "Returns string type for a service object of type '<GetControlMode-response>"
  "dsr_msgs/GetControlModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControlMode-response)))
  "Returns string type for a service object of type 'GetControlMode-response"
  "dsr_msgs/GetControlModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControlMode-response>)))
  "Returns md5sum for a message object of type '<GetControlMode-response>"
  "bc4e735fc2aafec8676fb72eaa5429bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControlMode-response)))
  "Returns md5sum for a message object of type 'GetControlMode-response"
  "bc4e735fc2aafec8676fb72eaa5429bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControlMode-response>)))
  "Returns full string definition for message of type '<GetControlMode-response>"
  (cl:format cl:nil "int8    control_mode        # Control mode : Position control mode(3), Torque control mode(4)~%bool    success ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControlMode-response)))
  "Returns full string definition for message of type 'GetControlMode-response"
  (cl:format cl:nil "int8    control_mode        # Control mode : Position control mode(3), Torque control mode(4)~%bool    success ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControlMode-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControlMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControlMode-response
    (cl:cons ':control_mode (control_mode msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetControlMode)))
  'GetControlMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetControlMode)))
  'GetControlMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControlMode)))
  "Returns string type for a service object of type '<GetControlMode>"
  "dsr_msgs/GetControlMode")