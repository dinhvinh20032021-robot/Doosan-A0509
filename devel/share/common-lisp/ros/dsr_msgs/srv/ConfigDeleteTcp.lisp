; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude ConfigDeleteTcp-request.msg.html

(cl:defclass <ConfigDeleteTcp-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass ConfigDeleteTcp-request (<ConfigDeleteTcp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigDeleteTcp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigDeleteTcp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ConfigDeleteTcp-request> is deprecated: use dsr_msgs-srv:ConfigDeleteTcp-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ConfigDeleteTcp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:name-val is deprecated.  Use dsr_msgs-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigDeleteTcp-request>) ostream)
  "Serializes a message object of type '<ConfigDeleteTcp-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigDeleteTcp-request>) istream)
  "Deserializes a message object of type '<ConfigDeleteTcp-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigDeleteTcp-request>)))
  "Returns string type for a service object of type '<ConfigDeleteTcp-request>"
  "dsr_msgs/ConfigDeleteTcpRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigDeleteTcp-request)))
  "Returns string type for a service object of type 'ConfigDeleteTcp-request"
  "dsr_msgs/ConfigDeleteTcpRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigDeleteTcp-request>)))
  "Returns md5sum for a message object of type '<ConfigDeleteTcp-request>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigDeleteTcp-request)))
  "Returns md5sum for a message object of type 'ConfigDeleteTcp-request"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigDeleteTcp-request>)))
  "Returns full string definition for message of type '<ConfigDeleteTcp-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# config_delete_tcp  ~%# It is a service for deleting the TCP information registered in advance in the robot controller~%#____________________________________________________________________________________________~%~%string          name             # tcp name ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigDeleteTcp-request)))
  "Returns full string definition for message of type 'ConfigDeleteTcp-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# config_delete_tcp  ~%# It is a service for deleting the TCP information registered in advance in the robot controller~%#____________________________________________________________________________________________~%~%string          name             # tcp name ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigDeleteTcp-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigDeleteTcp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigDeleteTcp-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude ConfigDeleteTcp-response.msg.html

(cl:defclass <ConfigDeleteTcp-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ConfigDeleteTcp-response (<ConfigDeleteTcp-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigDeleteTcp-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigDeleteTcp-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ConfigDeleteTcp-response> is deprecated: use dsr_msgs-srv:ConfigDeleteTcp-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ConfigDeleteTcp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigDeleteTcp-response>) ostream)
  "Serializes a message object of type '<ConfigDeleteTcp-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigDeleteTcp-response>) istream)
  "Deserializes a message object of type '<ConfigDeleteTcp-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigDeleteTcp-response>)))
  "Returns string type for a service object of type '<ConfigDeleteTcp-response>"
  "dsr_msgs/ConfigDeleteTcpResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigDeleteTcp-response)))
  "Returns string type for a service object of type 'ConfigDeleteTcp-response"
  "dsr_msgs/ConfigDeleteTcpResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigDeleteTcp-response>)))
  "Returns md5sum for a message object of type '<ConfigDeleteTcp-response>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigDeleteTcp-response)))
  "Returns md5sum for a message object of type 'ConfigDeleteTcp-response"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigDeleteTcp-response>)))
  "Returns full string definition for message of type '<ConfigDeleteTcp-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigDeleteTcp-response)))
  "Returns full string definition for message of type 'ConfigDeleteTcp-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigDeleteTcp-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigDeleteTcp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigDeleteTcp-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConfigDeleteTcp)))
  'ConfigDeleteTcp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConfigDeleteTcp)))
  'ConfigDeleteTcp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigDeleteTcp)))
  "Returns string type for a service object of type '<ConfigDeleteTcp>"
  "dsr_msgs/ConfigDeleteTcp")