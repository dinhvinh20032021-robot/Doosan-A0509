; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetSafeStopResetType-request.msg.html

(cl:defclass <SetSafeStopResetType-request> (roslisp-msg-protocol:ros-message)
  ((reset_type
    :reader reset_type
    :initarg :reset_type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetSafeStopResetType-request (<SetSafeStopResetType-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSafeStopResetType-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSafeStopResetType-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetSafeStopResetType-request> is deprecated: use dsr_msgs-srv:SetSafeStopResetType-request instead.")))

(cl:ensure-generic-function 'reset_type-val :lambda-list '(m))
(cl:defmethod reset_type-val ((m <SetSafeStopResetType-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:reset_type-val is deprecated.  Use dsr_msgs-srv:reset_type instead.")
  (reset_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSafeStopResetType-request>) ostream)
  "Serializes a message object of type '<SetSafeStopResetType-request>"
  (cl:let* ((signed (cl:slot-value msg 'reset_type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSafeStopResetType-request>) istream)
  "Deserializes a message object of type '<SetSafeStopResetType-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reset_type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSafeStopResetType-request>)))
  "Returns string type for a service object of type '<SetSafeStopResetType-request>"
  "dsr_msgs/SetSafeStopResetTypeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafeStopResetType-request)))
  "Returns string type for a service object of type 'SetSafeStopResetType-request"
  "dsr_msgs/SetSafeStopResetTypeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSafeStopResetType-request>)))
  "Returns md5sum for a message object of type '<SetSafeStopResetType-request>"
  "956a2bdbbc2e1317a58568aa311b17b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSafeStopResetType-request)))
  "Returns md5sum for a message object of type 'SetSafeStopResetType-request"
  "956a2bdbbc2e1317a58568aa311b17b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSafeStopResetType-request>)))
  "Returns full string definition for message of type '<SetSafeStopResetType-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_safe_stop_reset_type~%#____________________________________________________________________________________________~%~%int8 reset_type     # 0=SAFE_STOP_RESET_TYPE_DEFAULT = SAFE_STOP_RESET_TYPE_PROGRAM_STOP , 1= SAFE_STOP_RESET_TYPE_PROGRAM_RESUME ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSafeStopResetType-request)))
  "Returns full string definition for message of type 'SetSafeStopResetType-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_safe_stop_reset_type~%#____________________________________________________________________________________________~%~%int8 reset_type     # 0=SAFE_STOP_RESET_TYPE_DEFAULT = SAFE_STOP_RESET_TYPE_PROGRAM_STOP , 1= SAFE_STOP_RESET_TYPE_PROGRAM_RESUME ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSafeStopResetType-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSafeStopResetType-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSafeStopResetType-request
    (cl:cons ':reset_type (reset_type msg))
))
;//! \htmlinclude SetSafeStopResetType-response.msg.html

(cl:defclass <SetSafeStopResetType-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetSafeStopResetType-response (<SetSafeStopResetType-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSafeStopResetType-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSafeStopResetType-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetSafeStopResetType-response> is deprecated: use dsr_msgs-srv:SetSafeStopResetType-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetSafeStopResetType-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSafeStopResetType-response>) ostream)
  "Serializes a message object of type '<SetSafeStopResetType-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSafeStopResetType-response>) istream)
  "Deserializes a message object of type '<SetSafeStopResetType-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSafeStopResetType-response>)))
  "Returns string type for a service object of type '<SetSafeStopResetType-response>"
  "dsr_msgs/SetSafeStopResetTypeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafeStopResetType-response)))
  "Returns string type for a service object of type 'SetSafeStopResetType-response"
  "dsr_msgs/SetSafeStopResetTypeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSafeStopResetType-response>)))
  "Returns md5sum for a message object of type '<SetSafeStopResetType-response>"
  "956a2bdbbc2e1317a58568aa311b17b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSafeStopResetType-response)))
  "Returns md5sum for a message object of type 'SetSafeStopResetType-response"
  "956a2bdbbc2e1317a58568aa311b17b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSafeStopResetType-response>)))
  "Returns full string definition for message of type '<SetSafeStopResetType-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSafeStopResetType-response)))
  "Returns full string definition for message of type 'SetSafeStopResetType-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSafeStopResetType-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSafeStopResetType-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSafeStopResetType-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetSafeStopResetType)))
  'SetSafeStopResetType-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetSafeStopResetType)))
  'SetSafeStopResetType-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafeStopResetType)))
  "Returns string type for a service object of type '<SetSafeStopResetType>"
  "dsr_msgs/SetSafeStopResetType")