; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude MoveResume-request.msg.html

(cl:defclass <MoveResume-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MoveResume-request (<MoveResume-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveResume-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveResume-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<MoveResume-request> is deprecated: use dsr_msgs-srv:MoveResume-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveResume-request>) ostream)
  "Serializes a message object of type '<MoveResume-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveResume-request>) istream)
  "Deserializes a message object of type '<MoveResume-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveResume-request>)))
  "Returns string type for a service object of type '<MoveResume-request>"
  "dsr_msgs/MoveResumeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveResume-request)))
  "Returns string type for a service object of type 'MoveResume-request"
  "dsr_msgs/MoveResumeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveResume-request>)))
  "Returns md5sum for a message object of type '<MoveResume-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveResume-request)))
  "Returns md5sum for a message object of type 'MoveResume-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveResume-request>)))
  "Returns full string definition for message of type '<MoveResume-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# motion pause~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveResume-request)))
  "Returns full string definition for message of type 'MoveResume-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# motion pause~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveResume-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveResume-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveResume-request
))
;//! \htmlinclude MoveResume-response.msg.html

(cl:defclass <MoveResume-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoveResume-response (<MoveResume-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveResume-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveResume-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<MoveResume-response> is deprecated: use dsr_msgs-srv:MoveResume-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <MoveResume-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveResume-response>) ostream)
  "Serializes a message object of type '<MoveResume-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveResume-response>) istream)
  "Deserializes a message object of type '<MoveResume-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveResume-response>)))
  "Returns string type for a service object of type '<MoveResume-response>"
  "dsr_msgs/MoveResumeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveResume-response)))
  "Returns string type for a service object of type 'MoveResume-response"
  "dsr_msgs/MoveResumeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveResume-response>)))
  "Returns md5sum for a message object of type '<MoveResume-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveResume-response)))
  "Returns md5sum for a message object of type 'MoveResume-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveResume-response>)))
  "Returns full string definition for message of type '<MoveResume-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveResume-response)))
  "Returns full string definition for message of type 'MoveResume-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveResume-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveResume-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveResume-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveResume)))
  'MoveResume-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveResume)))
  'MoveResume-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveResume)))
  "Returns string type for a service object of type '<MoveResume>"
  "dsr_msgs/MoveResume")