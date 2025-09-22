; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude Robotiq2FClose-request.msg.html

(cl:defclass <Robotiq2FClose-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Robotiq2FClose-request (<Robotiq2FClose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Robotiq2FClose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Robotiq2FClose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<Robotiq2FClose-request> is deprecated: use dsr_msgs-srv:Robotiq2FClose-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Robotiq2FClose-request>) ostream)
  "Serializes a message object of type '<Robotiq2FClose-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Robotiq2FClose-request>) istream)
  "Deserializes a message object of type '<Robotiq2FClose-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Robotiq2FClose-request>)))
  "Returns string type for a service object of type '<Robotiq2FClose-request>"
  "dsr_msgs/Robotiq2FCloseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robotiq2FClose-request)))
  "Returns string type for a service object of type 'Robotiq2FClose-request"
  "dsr_msgs/Robotiq2FCloseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Robotiq2FClose-request>)))
  "Returns md5sum for a message object of type '<Robotiq2FClose-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Robotiq2FClose-request)))
  "Returns md5sum for a message object of type 'Robotiq2FClose-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Robotiq2FClose-request>)))
  "Returns full string definition for message of type '<Robotiq2FClose-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# robotiq_2f_close~%# close the virtual robotiq gripper  ~%#____________________________________________________________________________________________~%~%# None     ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Robotiq2FClose-request)))
  "Returns full string definition for message of type 'Robotiq2FClose-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# robotiq_2f_close~%# close the virtual robotiq gripper  ~%#____________________________________________________________________________________________~%~%# None     ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Robotiq2FClose-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Robotiq2FClose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Robotiq2FClose-request
))
;//! \htmlinclude Robotiq2FClose-response.msg.html

(cl:defclass <Robotiq2FClose-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Robotiq2FClose-response (<Robotiq2FClose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Robotiq2FClose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Robotiq2FClose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<Robotiq2FClose-response> is deprecated: use dsr_msgs-srv:Robotiq2FClose-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Robotiq2FClose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Robotiq2FClose-response>) ostream)
  "Serializes a message object of type '<Robotiq2FClose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Robotiq2FClose-response>) istream)
  "Deserializes a message object of type '<Robotiq2FClose-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Robotiq2FClose-response>)))
  "Returns string type for a service object of type '<Robotiq2FClose-response>"
  "dsr_msgs/Robotiq2FCloseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robotiq2FClose-response)))
  "Returns string type for a service object of type 'Robotiq2FClose-response"
  "dsr_msgs/Robotiq2FCloseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Robotiq2FClose-response>)))
  "Returns md5sum for a message object of type '<Robotiq2FClose-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Robotiq2FClose-response)))
  "Returns md5sum for a message object of type 'Robotiq2FClose-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Robotiq2FClose-response>)))
  "Returns full string definition for message of type '<Robotiq2FClose-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Robotiq2FClose-response)))
  "Returns full string definition for message of type 'Robotiq2FClose-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Robotiq2FClose-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Robotiq2FClose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Robotiq2FClose-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Robotiq2FClose)))
  'Robotiq2FClose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Robotiq2FClose)))
  'Robotiq2FClose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robotiq2FClose)))
  "Returns string type for a service object of type '<Robotiq2FClose>"
  "dsr_msgs/Robotiq2FClose")