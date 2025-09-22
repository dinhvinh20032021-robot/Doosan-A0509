; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude ManageAccessControl-request.msg.html

(cl:defclass <ManageAccessControl-request> (roslisp-msg-protocol:ros-message)
  ((access_control
    :reader access_control
    :initarg :access_control
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ManageAccessControl-request (<ManageAccessControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ManageAccessControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ManageAccessControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ManageAccessControl-request> is deprecated: use dsr_msgs-srv:ManageAccessControl-request instead.")))

(cl:ensure-generic-function 'access_control-val :lambda-list '(m))
(cl:defmethod access_control-val ((m <ManageAccessControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:access_control-val is deprecated.  Use dsr_msgs-srv:access_control instead.")
  (access_control m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ManageAccessControl-request>) ostream)
  "Serializes a message object of type '<ManageAccessControl-request>"
  (cl:let* ((signed (cl:slot-value msg 'access_control)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ManageAccessControl-request>) istream)
  "Deserializes a message object of type '<ManageAccessControl-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'access_control) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ManageAccessControl-request>)))
  "Returns string type for a service object of type '<ManageAccessControl-request>"
  "dsr_msgs/ManageAccessControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ManageAccessControl-request)))
  "Returns string type for a service object of type 'ManageAccessControl-request"
  "dsr_msgs/ManageAccessControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ManageAccessControl-request>)))
  "Returns md5sum for a message object of type '<ManageAccessControl-request>"
  "73b144debe54a1756145c8c37ae08b64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ManageAccessControl-request)))
  "Returns md5sum for a message object of type 'ManageAccessControl-request"
  "73b144debe54a1756145c8c37ae08b64")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ManageAccessControl-request>)))
  "Returns full string definition for message of type '<ManageAccessControl-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# manage_access_control~%#____________________________________________________________________________________________~%~%int8 access_control  # 0 : MANAGE_ACCESS_CONTROL_FORCE_REQUEST~%                     # 1 : MANAGE_ACCESS_CONTROL_REQUEST~%                     # 2 : MANAGE_ACCESS_CONTROL_RESPONSE_YES~%                     # 3 : MANAGE_ACCESS_CONTROL_RESPONSE_NO~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ManageAccessControl-request)))
  "Returns full string definition for message of type 'ManageAccessControl-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# manage_access_control~%#____________________________________________________________________________________________~%~%int8 access_control  # 0 : MANAGE_ACCESS_CONTROL_FORCE_REQUEST~%                     # 1 : MANAGE_ACCESS_CONTROL_REQUEST~%                     # 2 : MANAGE_ACCESS_CONTROL_RESPONSE_YES~%                     # 3 : MANAGE_ACCESS_CONTROL_RESPONSE_NO~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ManageAccessControl-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ManageAccessControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ManageAccessControl-request
    (cl:cons ':access_control (access_control msg))
))
;//! \htmlinclude ManageAccessControl-response.msg.html

(cl:defclass <ManageAccessControl-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ManageAccessControl-response (<ManageAccessControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ManageAccessControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ManageAccessControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ManageAccessControl-response> is deprecated: use dsr_msgs-srv:ManageAccessControl-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ManageAccessControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ManageAccessControl-response>) ostream)
  "Serializes a message object of type '<ManageAccessControl-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ManageAccessControl-response>) istream)
  "Deserializes a message object of type '<ManageAccessControl-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ManageAccessControl-response>)))
  "Returns string type for a service object of type '<ManageAccessControl-response>"
  "dsr_msgs/ManageAccessControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ManageAccessControl-response)))
  "Returns string type for a service object of type 'ManageAccessControl-response"
  "dsr_msgs/ManageAccessControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ManageAccessControl-response>)))
  "Returns md5sum for a message object of type '<ManageAccessControl-response>"
  "73b144debe54a1756145c8c37ae08b64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ManageAccessControl-response)))
  "Returns md5sum for a message object of type 'ManageAccessControl-response"
  "73b144debe54a1756145c8c37ae08b64")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ManageAccessControl-response>)))
  "Returns full string definition for message of type '<ManageAccessControl-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ManageAccessControl-response)))
  "Returns full string definition for message of type 'ManageAccessControl-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ManageAccessControl-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ManageAccessControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ManageAccessControl-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ManageAccessControl)))
  'ManageAccessControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ManageAccessControl)))
  'ManageAccessControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ManageAccessControl)))
  "Returns string type for a service object of type '<ManageAccessControl>"
  "dsr_msgs/ManageAccessControl")