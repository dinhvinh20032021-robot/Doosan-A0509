; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude MovePause-request.msg.html

(cl:defclass <MovePause-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MovePause-request (<MovePause-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MovePause-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MovePause-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<MovePause-request> is deprecated: use dsr_msgs-srv:MovePause-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MovePause-request>) ostream)
  "Serializes a message object of type '<MovePause-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MovePause-request>) istream)
  "Deserializes a message object of type '<MovePause-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MovePause-request>)))
  "Returns string type for a service object of type '<MovePause-request>"
  "dsr_msgs/MovePauseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MovePause-request)))
  "Returns string type for a service object of type 'MovePause-request"
  "dsr_msgs/MovePauseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MovePause-request>)))
  "Returns md5sum for a message object of type '<MovePause-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MovePause-request)))
  "Returns md5sum for a message object of type 'MovePause-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MovePause-request>)))
  "Returns full string definition for message of type '<MovePause-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# motion pause~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MovePause-request)))
  "Returns full string definition for message of type 'MovePause-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# motion pause~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MovePause-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MovePause-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MovePause-request
))
;//! \htmlinclude MovePause-response.msg.html

(cl:defclass <MovePause-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MovePause-response (<MovePause-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MovePause-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MovePause-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<MovePause-response> is deprecated: use dsr_msgs-srv:MovePause-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <MovePause-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MovePause-response>) ostream)
  "Serializes a message object of type '<MovePause-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MovePause-response>) istream)
  "Deserializes a message object of type '<MovePause-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MovePause-response>)))
  "Returns string type for a service object of type '<MovePause-response>"
  "dsr_msgs/MovePauseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MovePause-response)))
  "Returns string type for a service object of type 'MovePause-response"
  "dsr_msgs/MovePauseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MovePause-response>)))
  "Returns md5sum for a message object of type '<MovePause-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MovePause-response)))
  "Returns md5sum for a message object of type 'MovePause-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MovePause-response>)))
  "Returns full string definition for message of type '<MovePause-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MovePause-response)))
  "Returns full string definition for message of type 'MovePause-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MovePause-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MovePause-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MovePause-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MovePause)))
  'MovePause-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MovePause)))
  'MovePause-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MovePause)))
  "Returns string type for a service object of type '<MovePause>"
  "dsr_msgs/MovePause")