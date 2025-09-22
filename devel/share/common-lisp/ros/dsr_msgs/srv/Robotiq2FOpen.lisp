; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude Robotiq2FOpen-request.msg.html

(cl:defclass <Robotiq2FOpen-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Robotiq2FOpen-request (<Robotiq2FOpen-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Robotiq2FOpen-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Robotiq2FOpen-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<Robotiq2FOpen-request> is deprecated: use dsr_msgs-srv:Robotiq2FOpen-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Robotiq2FOpen-request>) ostream)
  "Serializes a message object of type '<Robotiq2FOpen-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Robotiq2FOpen-request>) istream)
  "Deserializes a message object of type '<Robotiq2FOpen-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Robotiq2FOpen-request>)))
  "Returns string type for a service object of type '<Robotiq2FOpen-request>"
  "dsr_msgs/Robotiq2FOpenRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robotiq2FOpen-request)))
  "Returns string type for a service object of type 'Robotiq2FOpen-request"
  "dsr_msgs/Robotiq2FOpenRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Robotiq2FOpen-request>)))
  "Returns md5sum for a message object of type '<Robotiq2FOpen-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Robotiq2FOpen-request)))
  "Returns md5sum for a message object of type 'Robotiq2FOpen-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Robotiq2FOpen-request>)))
  "Returns full string definition for message of type '<Robotiq2FOpen-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# robotiq_2f_open~%# open the virtual robotiq gripper  ~%#____________________________________________________________________________________________~%~%# None  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Robotiq2FOpen-request)))
  "Returns full string definition for message of type 'Robotiq2FOpen-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# robotiq_2f_open~%# open the virtual robotiq gripper  ~%#____________________________________________________________________________________________~%~%# None  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Robotiq2FOpen-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Robotiq2FOpen-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Robotiq2FOpen-request
))
;//! \htmlinclude Robotiq2FOpen-response.msg.html

(cl:defclass <Robotiq2FOpen-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Robotiq2FOpen-response (<Robotiq2FOpen-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Robotiq2FOpen-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Robotiq2FOpen-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<Robotiq2FOpen-response> is deprecated: use dsr_msgs-srv:Robotiq2FOpen-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Robotiq2FOpen-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Robotiq2FOpen-response>) ostream)
  "Serializes a message object of type '<Robotiq2FOpen-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Robotiq2FOpen-response>) istream)
  "Deserializes a message object of type '<Robotiq2FOpen-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Robotiq2FOpen-response>)))
  "Returns string type for a service object of type '<Robotiq2FOpen-response>"
  "dsr_msgs/Robotiq2FOpenResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robotiq2FOpen-response)))
  "Returns string type for a service object of type 'Robotiq2FOpen-response"
  "dsr_msgs/Robotiq2FOpenResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Robotiq2FOpen-response>)))
  "Returns md5sum for a message object of type '<Robotiq2FOpen-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Robotiq2FOpen-response)))
  "Returns md5sum for a message object of type 'Robotiq2FOpen-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Robotiq2FOpen-response>)))
  "Returns full string definition for message of type '<Robotiq2FOpen-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Robotiq2FOpen-response)))
  "Returns full string definition for message of type 'Robotiq2FOpen-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Robotiq2FOpen-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Robotiq2FOpen-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Robotiq2FOpen-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Robotiq2FOpen)))
  'Robotiq2FOpen-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Robotiq2FOpen)))
  'Robotiq2FOpen-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robotiq2FOpen)))
  "Returns string type for a service object of type '<Robotiq2FOpen>"
  "dsr_msgs/Robotiq2FOpen")