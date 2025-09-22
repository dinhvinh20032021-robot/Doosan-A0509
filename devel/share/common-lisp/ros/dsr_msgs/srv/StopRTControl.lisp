; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude StopRTControl-request.msg.html

(cl:defclass <StopRTControl-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StopRTControl-request (<StopRTControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopRTControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopRTControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<StopRTControl-request> is deprecated: use dsr_msgs-srv:StopRTControl-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopRTControl-request>) ostream)
  "Serializes a message object of type '<StopRTControl-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopRTControl-request>) istream)
  "Deserializes a message object of type '<StopRTControl-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopRTControl-request>)))
  "Returns string type for a service object of type '<StopRTControl-request>"
  "dsr_msgs/StopRTControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopRTControl-request)))
  "Returns string type for a service object of type 'StopRTControl-request"
  "dsr_msgs/StopRTControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopRTControl-request>)))
  "Returns md5sum for a message object of type '<StopRTControl-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopRTControl-request)))
  "Returns md5sum for a message object of type 'StopRTControl-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopRTControl-request>)))
  "Returns full string definition for message of type '<StopRTControl-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# stop_rt_control~%#____________________________________________________________________________________________~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopRTControl-request)))
  "Returns full string definition for message of type 'StopRTControl-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# stop_rt_control~%#____________________________________________________________________________________________~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopRTControl-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopRTControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StopRTControl-request
))
;//! \htmlinclude StopRTControl-response.msg.html

(cl:defclass <StopRTControl-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass StopRTControl-response (<StopRTControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopRTControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopRTControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<StopRTControl-response> is deprecated: use dsr_msgs-srv:StopRTControl-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <StopRTControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopRTControl-response>) ostream)
  "Serializes a message object of type '<StopRTControl-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopRTControl-response>) istream)
  "Deserializes a message object of type '<StopRTControl-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopRTControl-response>)))
  "Returns string type for a service object of type '<StopRTControl-response>"
  "dsr_msgs/StopRTControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopRTControl-response)))
  "Returns string type for a service object of type 'StopRTControl-response"
  "dsr_msgs/StopRTControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopRTControl-response>)))
  "Returns md5sum for a message object of type '<StopRTControl-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopRTControl-response)))
  "Returns md5sum for a message object of type 'StopRTControl-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopRTControl-response>)))
  "Returns full string definition for message of type '<StopRTControl-response>"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopRTControl-response)))
  "Returns full string definition for message of type 'StopRTControl-response"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopRTControl-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopRTControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StopRTControl-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StopRTControl)))
  'StopRTControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StopRTControl)))
  'StopRTControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopRTControl)))
  "Returns string type for a service object of type '<StopRTControl>"
  "dsr_msgs/StopRTControl")