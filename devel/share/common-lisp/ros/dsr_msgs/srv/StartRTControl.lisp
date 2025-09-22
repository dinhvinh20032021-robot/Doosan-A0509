; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude StartRTControl-request.msg.html

(cl:defclass <StartRTControl-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StartRTControl-request (<StartRTControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartRTControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartRTControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<StartRTControl-request> is deprecated: use dsr_msgs-srv:StartRTControl-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartRTControl-request>) ostream)
  "Serializes a message object of type '<StartRTControl-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartRTControl-request>) istream)
  "Deserializes a message object of type '<StartRTControl-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartRTControl-request>)))
  "Returns string type for a service object of type '<StartRTControl-request>"
  "dsr_msgs/StartRTControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartRTControl-request)))
  "Returns string type for a service object of type 'StartRTControl-request"
  "dsr_msgs/StartRTControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartRTControl-request>)))
  "Returns md5sum for a message object of type '<StartRTControl-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartRTControl-request)))
  "Returns md5sum for a message object of type 'StartRTControl-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartRTControl-request>)))
  "Returns full string definition for message of type '<StartRTControl-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# start_rt_control~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartRTControl-request)))
  "Returns full string definition for message of type 'StartRTControl-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# start_rt_control~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartRTControl-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartRTControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartRTControl-request
))
;//! \htmlinclude StartRTControl-response.msg.html

(cl:defclass <StartRTControl-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass StartRTControl-response (<StartRTControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartRTControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartRTControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<StartRTControl-response> is deprecated: use dsr_msgs-srv:StartRTControl-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <StartRTControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartRTControl-response>) ostream)
  "Serializes a message object of type '<StartRTControl-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartRTControl-response>) istream)
  "Deserializes a message object of type '<StartRTControl-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartRTControl-response>)))
  "Returns string type for a service object of type '<StartRTControl-response>"
  "dsr_msgs/StartRTControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartRTControl-response)))
  "Returns string type for a service object of type 'StartRTControl-response"
  "dsr_msgs/StartRTControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartRTControl-response>)))
  "Returns md5sum for a message object of type '<StartRTControl-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartRTControl-response)))
  "Returns md5sum for a message object of type 'StartRTControl-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartRTControl-response>)))
  "Returns full string definition for message of type '<StartRTControl-response>"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartRTControl-response)))
  "Returns full string definition for message of type 'StartRTControl-response"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartRTControl-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartRTControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartRTControl-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartRTControl)))
  'StartRTControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartRTControl)))
  'StartRTControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartRTControl)))
  "Returns string type for a service object of type '<StartRTControl>"
  "dsr_msgs/StartRTControl")