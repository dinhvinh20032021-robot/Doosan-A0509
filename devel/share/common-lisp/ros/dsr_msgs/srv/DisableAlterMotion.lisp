; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude DisableAlterMotion-request.msg.html

(cl:defclass <DisableAlterMotion-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DisableAlterMotion-request (<DisableAlterMotion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DisableAlterMotion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DisableAlterMotion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<DisableAlterMotion-request> is deprecated: use dsr_msgs-srv:DisableAlterMotion-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DisableAlterMotion-request>) ostream)
  "Serializes a message object of type '<DisableAlterMotion-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DisableAlterMotion-request>) istream)
  "Deserializes a message object of type '<DisableAlterMotion-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DisableAlterMotion-request>)))
  "Returns string type for a service object of type '<DisableAlterMotion-request>"
  "dsr_msgs/DisableAlterMotionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisableAlterMotion-request)))
  "Returns string type for a service object of type 'DisableAlterMotion-request"
  "dsr_msgs/DisableAlterMotionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DisableAlterMotion-request>)))
  "Returns md5sum for a message object of type '<DisableAlterMotion-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DisableAlterMotion-request)))
  "Returns md5sum for a message object of type 'DisableAlterMotion-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DisableAlterMotion-request>)))
  "Returns full string definition for message of type '<DisableAlterMotion-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# disable_alter_motion  ~%# deactivates alter motion~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DisableAlterMotion-request)))
  "Returns full string definition for message of type 'DisableAlterMotion-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# disable_alter_motion  ~%# deactivates alter motion~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DisableAlterMotion-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DisableAlterMotion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DisableAlterMotion-request
))
;//! \htmlinclude DisableAlterMotion-response.msg.html

(cl:defclass <DisableAlterMotion-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DisableAlterMotion-response (<DisableAlterMotion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DisableAlterMotion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DisableAlterMotion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<DisableAlterMotion-response> is deprecated: use dsr_msgs-srv:DisableAlterMotion-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DisableAlterMotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DisableAlterMotion-response>) ostream)
  "Serializes a message object of type '<DisableAlterMotion-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DisableAlterMotion-response>) istream)
  "Deserializes a message object of type '<DisableAlterMotion-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DisableAlterMotion-response>)))
  "Returns string type for a service object of type '<DisableAlterMotion-response>"
  "dsr_msgs/DisableAlterMotionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisableAlterMotion-response)))
  "Returns string type for a service object of type 'DisableAlterMotion-response"
  "dsr_msgs/DisableAlterMotionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DisableAlterMotion-response>)))
  "Returns md5sum for a message object of type '<DisableAlterMotion-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DisableAlterMotion-response)))
  "Returns md5sum for a message object of type 'DisableAlterMotion-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DisableAlterMotion-response>)))
  "Returns full string definition for message of type '<DisableAlterMotion-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DisableAlterMotion-response)))
  "Returns full string definition for message of type 'DisableAlterMotion-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DisableAlterMotion-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DisableAlterMotion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DisableAlterMotion-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DisableAlterMotion)))
  'DisableAlterMotion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DisableAlterMotion)))
  'DisableAlterMotion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisableAlterMotion)))
  "Returns string type for a service object of type '<DisableAlterMotion>"
  "dsr_msgs/DisableAlterMotion")