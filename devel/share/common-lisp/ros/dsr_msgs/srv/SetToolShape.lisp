; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetToolShape-request.msg.html

(cl:defclass <SetToolShape-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass SetToolShape-request (<SetToolShape-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetToolShape-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetToolShape-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetToolShape-request> is deprecated: use dsr_msgs-srv:SetToolShape-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <SetToolShape-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:name-val is deprecated.  Use dsr_msgs-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetToolShape-request>) ostream)
  "Serializes a message object of type '<SetToolShape-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetToolShape-request>) istream)
  "Deserializes a message object of type '<SetToolShape-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetToolShape-request>)))
  "Returns string type for a service object of type '<SetToolShape-request>"
  "dsr_msgs/SetToolShapeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetToolShape-request)))
  "Returns string type for a service object of type 'SetToolShape-request"
  "dsr_msgs/SetToolShapeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetToolShape-request>)))
  "Returns md5sum for a message object of type '<SetToolShape-request>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetToolShape-request)))
  "Returns md5sum for a message object of type 'SetToolShape-request"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetToolShape-request>)))
  "Returns full string definition for message of type '<SetToolShape-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_tool_shape~%# It is a service to set information about currently installed tool.  ~%# Activates the tool shape information of the entered name among the tool shape information registered in the Teach Pendant~%#____________________________________________________________________________________________~%~%string          name        # Tool name registered in the Teach Pendant~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetToolShape-request)))
  "Returns full string definition for message of type 'SetToolShape-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_tool_shape~%# It is a service to set information about currently installed tool.  ~%# Activates the tool shape information of the entered name among the tool shape information registered in the Teach Pendant~%#____________________________________________________________________________________________~%~%string          name        # Tool name registered in the Teach Pendant~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetToolShape-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetToolShape-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetToolShape-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude SetToolShape-response.msg.html

(cl:defclass <SetToolShape-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetToolShape-response (<SetToolShape-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetToolShape-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetToolShape-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetToolShape-response> is deprecated: use dsr_msgs-srv:SetToolShape-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetToolShape-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetToolShape-response>) ostream)
  "Serializes a message object of type '<SetToolShape-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetToolShape-response>) istream)
  "Deserializes a message object of type '<SetToolShape-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetToolShape-response>)))
  "Returns string type for a service object of type '<SetToolShape-response>"
  "dsr_msgs/SetToolShapeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetToolShape-response)))
  "Returns string type for a service object of type 'SetToolShape-response"
  "dsr_msgs/SetToolShapeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetToolShape-response>)))
  "Returns md5sum for a message object of type '<SetToolShape-response>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetToolShape-response)))
  "Returns md5sum for a message object of type 'SetToolShape-response"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetToolShape-response>)))
  "Returns full string definition for message of type '<SetToolShape-response>"
  (cl:format cl:nil "bool            success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetToolShape-response)))
  "Returns full string definition for message of type 'SetToolShape-response"
  (cl:format cl:nil "bool            success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetToolShape-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetToolShape-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetToolShape-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetToolShape)))
  'SetToolShape-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetToolShape)))
  'SetToolShape-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetToolShape)))
  "Returns string type for a service object of type '<SetToolShape>"
  "dsr_msgs/SetToolShape")