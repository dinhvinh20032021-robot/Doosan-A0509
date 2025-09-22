; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude ReleaseProtectiveStop-request.msg.html

(cl:defclass <ReleaseProtectiveStop-request> (roslisp-msg-protocol:ros-message)
  ((release_mode
    :reader release_mode
    :initarg :release_mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ReleaseProtectiveStop-request (<ReleaseProtectiveStop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReleaseProtectiveStop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReleaseProtectiveStop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ReleaseProtectiveStop-request> is deprecated: use dsr_msgs-srv:ReleaseProtectiveStop-request instead.")))

(cl:ensure-generic-function 'release_mode-val :lambda-list '(m))
(cl:defmethod release_mode-val ((m <ReleaseProtectiveStop-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:release_mode-val is deprecated.  Use dsr_msgs-srv:release_mode instead.")
  (release_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReleaseProtectiveStop-request>) ostream)
  "Serializes a message object of type '<ReleaseProtectiveStop-request>"
  (cl:let* ((signed (cl:slot-value msg 'release_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReleaseProtectiveStop-request>) istream)
  "Deserializes a message object of type '<ReleaseProtectiveStop-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'release_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReleaseProtectiveStop-request>)))
  "Returns string type for a service object of type '<ReleaseProtectiveStop-request>"
  "dsr_msgs/ReleaseProtectiveStopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReleaseProtectiveStop-request)))
  "Returns string type for a service object of type 'ReleaseProtectiveStop-request"
  "dsr_msgs/ReleaseProtectiveStopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReleaseProtectiveStop-request>)))
  "Returns md5sum for a message object of type '<ReleaseProtectiveStop-request>"
  "3cf8efd6d59d99783a077dda000b1bbb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReleaseProtectiveStop-request)))
  "Returns md5sum for a message object of type 'ReleaseProtectiveStop-request"
  "3cf8efd6d59d99783a077dda000b1bbb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReleaseProtectiveStop-request>)))
  "Returns full string definition for message of type '<ReleaseProtectiveStop-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# release_protective_stop~%#____________________________________________________________________________________________~%~%int8 release_mode     # 0=RELEASE_MODE_STOP, 1=RELEASE_MODE_RESUME, 2=RELEASE_MODE_RELEASE, 3=RELEASE_MODE_RESET~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReleaseProtectiveStop-request)))
  "Returns full string definition for message of type 'ReleaseProtectiveStop-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# release_protective_stop~%#____________________________________________________________________________________________~%~%int8 release_mode     # 0=RELEASE_MODE_STOP, 1=RELEASE_MODE_RESUME, 2=RELEASE_MODE_RELEASE, 3=RELEASE_MODE_RESET~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReleaseProtectiveStop-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReleaseProtectiveStop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReleaseProtectiveStop-request
    (cl:cons ':release_mode (release_mode msg))
))
;//! \htmlinclude ReleaseProtectiveStop-response.msg.html

(cl:defclass <ReleaseProtectiveStop-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ReleaseProtectiveStop-response (<ReleaseProtectiveStop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReleaseProtectiveStop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReleaseProtectiveStop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ReleaseProtectiveStop-response> is deprecated: use dsr_msgs-srv:ReleaseProtectiveStop-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ReleaseProtectiveStop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReleaseProtectiveStop-response>) ostream)
  "Serializes a message object of type '<ReleaseProtectiveStop-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReleaseProtectiveStop-response>) istream)
  "Deserializes a message object of type '<ReleaseProtectiveStop-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReleaseProtectiveStop-response>)))
  "Returns string type for a service object of type '<ReleaseProtectiveStop-response>"
  "dsr_msgs/ReleaseProtectiveStopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReleaseProtectiveStop-response)))
  "Returns string type for a service object of type 'ReleaseProtectiveStop-response"
  "dsr_msgs/ReleaseProtectiveStopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReleaseProtectiveStop-response>)))
  "Returns md5sum for a message object of type '<ReleaseProtectiveStop-response>"
  "3cf8efd6d59d99783a077dda000b1bbb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReleaseProtectiveStop-response)))
  "Returns md5sum for a message object of type 'ReleaseProtectiveStop-response"
  "3cf8efd6d59d99783a077dda000b1bbb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReleaseProtectiveStop-response>)))
  "Returns full string definition for message of type '<ReleaseProtectiveStop-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReleaseProtectiveStop-response)))
  "Returns full string definition for message of type 'ReleaseProtectiveStop-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReleaseProtectiveStop-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReleaseProtectiveStop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReleaseProtectiveStop-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReleaseProtectiveStop)))
  'ReleaseProtectiveStop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReleaseProtectiveStop)))
  'ReleaseProtectiveStop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReleaseProtectiveStop)))
  "Returns string type for a service object of type '<ReleaseProtectiveStop>"
  "dsr_msgs/ReleaseProtectiveStop")