; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetCurrentTool-request.msg.html

(cl:defclass <SetCurrentTool-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass SetCurrentTool-request (<SetCurrentTool-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCurrentTool-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCurrentTool-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetCurrentTool-request> is deprecated: use dsr_msgs-srv:SetCurrentTool-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <SetCurrentTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:name-val is deprecated.  Use dsr_msgs-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCurrentTool-request>) ostream)
  "Serializes a message object of type '<SetCurrentTool-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCurrentTool-request>) istream)
  "Deserializes a message object of type '<SetCurrentTool-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCurrentTool-request>)))
  "Returns string type for a service object of type '<SetCurrentTool-request>"
  "dsr_msgs/SetCurrentToolRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCurrentTool-request)))
  "Returns string type for a service object of type 'SetCurrentTool-request"
  "dsr_msgs/SetCurrentToolRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCurrentTool-request>)))
  "Returns md5sum for a message object of type '<SetCurrentTool-request>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCurrentTool-request)))
  "Returns md5sum for a message object of type 'SetCurrentTool-request"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCurrentTool-request>)))
  "Returns full string definition for message of type '<SetCurrentTool-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_current_tool~%# It is a service to set information about currently installed tool.  ~%#____________________________________________________________________________________________~%~%string          name        # tool name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCurrentTool-request)))
  "Returns full string definition for message of type 'SetCurrentTool-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_current_tool~%# It is a service to set information about currently installed tool.  ~%#____________________________________________________________________________________________~%~%string          name        # tool name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCurrentTool-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCurrentTool-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCurrentTool-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude SetCurrentTool-response.msg.html

(cl:defclass <SetCurrentTool-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetCurrentTool-response (<SetCurrentTool-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCurrentTool-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCurrentTool-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetCurrentTool-response> is deprecated: use dsr_msgs-srv:SetCurrentTool-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetCurrentTool-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCurrentTool-response>) ostream)
  "Serializes a message object of type '<SetCurrentTool-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCurrentTool-response>) istream)
  "Deserializes a message object of type '<SetCurrentTool-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCurrentTool-response>)))
  "Returns string type for a service object of type '<SetCurrentTool-response>"
  "dsr_msgs/SetCurrentToolResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCurrentTool-response)))
  "Returns string type for a service object of type 'SetCurrentTool-response"
  "dsr_msgs/SetCurrentToolResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCurrentTool-response>)))
  "Returns md5sum for a message object of type '<SetCurrentTool-response>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCurrentTool-response)))
  "Returns md5sum for a message object of type 'SetCurrentTool-response"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCurrentTool-response>)))
  "Returns full string definition for message of type '<SetCurrentTool-response>"
  (cl:format cl:nil "bool            success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCurrentTool-response)))
  "Returns full string definition for message of type 'SetCurrentTool-response"
  (cl:format cl:nil "bool            success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCurrentTool-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCurrentTool-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCurrentTool-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetCurrentTool)))
  'SetCurrentTool-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetCurrentTool)))
  'SetCurrentTool-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCurrentTool)))
  "Returns string type for a service object of type '<SetCurrentTool>"
  "dsr_msgs/SetCurrentTool")