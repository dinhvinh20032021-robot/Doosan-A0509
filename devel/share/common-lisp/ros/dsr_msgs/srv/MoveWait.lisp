; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude MoveWait-request.msg.html

(cl:defclass <MoveWait-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MoveWait-request (<MoveWait-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveWait-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveWait-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<MoveWait-request> is deprecated: use dsr_msgs-srv:MoveWait-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveWait-request>) ostream)
  "Serializes a message object of type '<MoveWait-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveWait-request>) istream)
  "Deserializes a message object of type '<MoveWait-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveWait-request>)))
  "Returns string type for a service object of type '<MoveWait-request>"
  "dsr_msgs/MoveWaitRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveWait-request)))
  "Returns string type for a service object of type 'MoveWait-request"
  "dsr_msgs/MoveWaitRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveWait-request>)))
  "Returns md5sum for a message object of type '<MoveWait-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveWait-request)))
  "Returns md5sum for a message object of type 'MoveWait-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveWait-request>)))
  "Returns full string definition for message of type '<MoveWait-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# move_wait~%# This Service sets the waiting time between the previous motion command ~%# and the motion command in the next line.~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveWait-request)))
  "Returns full string definition for message of type 'MoveWait-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# move_wait~%# This Service sets the waiting time between the previous motion command ~%# and the motion command in the next line.~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveWait-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveWait-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveWait-request
))
;//! \htmlinclude MoveWait-response.msg.html

(cl:defclass <MoveWait-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoveWait-response (<MoveWait-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveWait-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveWait-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<MoveWait-response> is deprecated: use dsr_msgs-srv:MoveWait-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <MoveWait-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveWait-response>) ostream)
  "Serializes a message object of type '<MoveWait-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveWait-response>) istream)
  "Deserializes a message object of type '<MoveWait-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveWait-response>)))
  "Returns string type for a service object of type '<MoveWait-response>"
  "dsr_msgs/MoveWaitResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveWait-response)))
  "Returns string type for a service object of type 'MoveWait-response"
  "dsr_msgs/MoveWaitResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveWait-response>)))
  "Returns md5sum for a message object of type '<MoveWait-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveWait-response)))
  "Returns md5sum for a message object of type 'MoveWait-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveWait-response>)))
  "Returns full string definition for message of type '<MoveWait-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveWait-response)))
  "Returns full string definition for message of type 'MoveWait-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveWait-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveWait-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveWait-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveWait)))
  'MoveWait-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveWait)))
  'MoveWait-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveWait)))
  "Returns string type for a service object of type '<MoveWait>"
  "dsr_msgs/MoveWait")