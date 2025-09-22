; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude ConfigDeleteTool-request.msg.html

(cl:defclass <ConfigDeleteTool-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass ConfigDeleteTool-request (<ConfigDeleteTool-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigDeleteTool-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigDeleteTool-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ConfigDeleteTool-request> is deprecated: use dsr_msgs-srv:ConfigDeleteTool-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ConfigDeleteTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:name-val is deprecated.  Use dsr_msgs-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigDeleteTool-request>) ostream)
  "Serializes a message object of type '<ConfigDeleteTool-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigDeleteTool-request>) istream)
  "Deserializes a message object of type '<ConfigDeleteTool-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigDeleteTool-request>)))
  "Returns string type for a service object of type '<ConfigDeleteTool-request>"
  "dsr_msgs/ConfigDeleteToolRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigDeleteTool-request)))
  "Returns string type for a service object of type 'ConfigDeleteTool-request"
  "dsr_msgs/ConfigDeleteToolRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigDeleteTool-request>)))
  "Returns md5sum for a message object of type '<ConfigDeleteTool-request>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigDeleteTool-request)))
  "Returns md5sum for a message object of type 'ConfigDeleteTool-request"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigDeleteTool-request>)))
  "Returns full string definition for message of type '<ConfigDeleteTool-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# config_delete_tool  ~%# It is a service to delete tool information registered in advance in the robot controller~%#____________________________________________________________________________________________~%~%string          name        # tool name ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigDeleteTool-request)))
  "Returns full string definition for message of type 'ConfigDeleteTool-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# config_delete_tool  ~%# It is a service to delete tool information registered in advance in the robot controller~%#____________________________________________________________________________________________~%~%string          name        # tool name ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigDeleteTool-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigDeleteTool-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigDeleteTool-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude ConfigDeleteTool-response.msg.html

(cl:defclass <ConfigDeleteTool-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ConfigDeleteTool-response (<ConfigDeleteTool-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigDeleteTool-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigDeleteTool-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ConfigDeleteTool-response> is deprecated: use dsr_msgs-srv:ConfigDeleteTool-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ConfigDeleteTool-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigDeleteTool-response>) ostream)
  "Serializes a message object of type '<ConfigDeleteTool-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigDeleteTool-response>) istream)
  "Deserializes a message object of type '<ConfigDeleteTool-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigDeleteTool-response>)))
  "Returns string type for a service object of type '<ConfigDeleteTool-response>"
  "dsr_msgs/ConfigDeleteToolResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigDeleteTool-response)))
  "Returns string type for a service object of type 'ConfigDeleteTool-response"
  "dsr_msgs/ConfigDeleteToolResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigDeleteTool-response>)))
  "Returns md5sum for a message object of type '<ConfigDeleteTool-response>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigDeleteTool-response)))
  "Returns md5sum for a message object of type 'ConfigDeleteTool-response"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigDeleteTool-response>)))
  "Returns full string definition for message of type '<ConfigDeleteTool-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigDeleteTool-response)))
  "Returns full string definition for message of type 'ConfigDeleteTool-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigDeleteTool-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigDeleteTool-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigDeleteTool-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConfigDeleteTool)))
  'ConfigDeleteTool-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConfigDeleteTool)))
  'ConfigDeleteTool-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigDeleteTool)))
  "Returns string type for a service object of type '<ConfigDeleteTool>"
  "dsr_msgs/ConfigDeleteTool")