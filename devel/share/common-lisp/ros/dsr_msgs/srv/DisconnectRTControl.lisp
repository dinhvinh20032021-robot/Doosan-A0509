; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude DisconnectRTControl-request.msg.html

(cl:defclass <DisconnectRTControl-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DisconnectRTControl-request (<DisconnectRTControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DisconnectRTControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DisconnectRTControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<DisconnectRTControl-request> is deprecated: use dsr_msgs-srv:DisconnectRTControl-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DisconnectRTControl-request>) ostream)
  "Serializes a message object of type '<DisconnectRTControl-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DisconnectRTControl-request>) istream)
  "Deserializes a message object of type '<DisconnectRTControl-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DisconnectRTControl-request>)))
  "Returns string type for a service object of type '<DisconnectRTControl-request>"
  "dsr_msgs/DisconnectRTControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisconnectRTControl-request)))
  "Returns string type for a service object of type 'DisconnectRTControl-request"
  "dsr_msgs/DisconnectRTControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DisconnectRTControl-request>)))
  "Returns md5sum for a message object of type '<DisconnectRTControl-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DisconnectRTControl-request)))
  "Returns md5sum for a message object of type 'DisconnectRTControl-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DisconnectRTControl-request>)))
  "Returns full string definition for message of type '<DisconnectRTControl-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# disconnect_rt_control~%#____________________________________________________________________________________________~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DisconnectRTControl-request)))
  "Returns full string definition for message of type 'DisconnectRTControl-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# disconnect_rt_control~%#____________________________________________________________________________________________~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DisconnectRTControl-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DisconnectRTControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DisconnectRTControl-request
))
;//! \htmlinclude DisconnectRTControl-response.msg.html

(cl:defclass <DisconnectRTControl-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DisconnectRTControl-response (<DisconnectRTControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DisconnectRTControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DisconnectRTControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<DisconnectRTControl-response> is deprecated: use dsr_msgs-srv:DisconnectRTControl-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DisconnectRTControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DisconnectRTControl-response>) ostream)
  "Serializes a message object of type '<DisconnectRTControl-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DisconnectRTControl-response>) istream)
  "Deserializes a message object of type '<DisconnectRTControl-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DisconnectRTControl-response>)))
  "Returns string type for a service object of type '<DisconnectRTControl-response>"
  "dsr_msgs/DisconnectRTControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisconnectRTControl-response)))
  "Returns string type for a service object of type 'DisconnectRTControl-response"
  "dsr_msgs/DisconnectRTControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DisconnectRTControl-response>)))
  "Returns md5sum for a message object of type '<DisconnectRTControl-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DisconnectRTControl-response)))
  "Returns md5sum for a message object of type 'DisconnectRTControl-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DisconnectRTControl-response>)))
  "Returns full string definition for message of type '<DisconnectRTControl-response>"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DisconnectRTControl-response)))
  "Returns full string definition for message of type 'DisconnectRTControl-response"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DisconnectRTControl-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DisconnectRTControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DisconnectRTControl-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DisconnectRTControl)))
  'DisconnectRTControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DisconnectRTControl)))
  'DisconnectRTControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisconnectRTControl)))
  "Returns string type for a service object of type '<DisconnectRTControl>"
  "dsr_msgs/DisconnectRTControl")