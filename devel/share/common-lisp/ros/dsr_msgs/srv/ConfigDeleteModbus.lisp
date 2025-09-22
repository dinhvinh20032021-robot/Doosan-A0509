; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude ConfigDeleteModbus-request.msg.html

(cl:defclass <ConfigDeleteModbus-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass ConfigDeleteModbus-request (<ConfigDeleteModbus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigDeleteModbus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigDeleteModbus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ConfigDeleteModbus-request> is deprecated: use dsr_msgs-srv:ConfigDeleteModbus-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ConfigDeleteModbus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:name-val is deprecated.  Use dsr_msgs-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigDeleteModbus-request>) ostream)
  "Serializes a message object of type '<ConfigDeleteModbus-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigDeleteModbus-request>) istream)
  "Deserializes a message object of type '<ConfigDeleteModbus-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigDeleteModbus-request>)))
  "Returns string type for a service object of type '<ConfigDeleteModbus-request>"
  "dsr_msgs/ConfigDeleteModbusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigDeleteModbus-request)))
  "Returns string type for a service object of type 'ConfigDeleteModbus-request"
  "dsr_msgs/ConfigDeleteModbusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigDeleteModbus-request>)))
  "Returns md5sum for a message object of type '<ConfigDeleteModbus-request>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigDeleteModbus-request)))
  "Returns md5sum for a message object of type 'ConfigDeleteModbus-request"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigDeleteModbus-request>)))
  "Returns full string definition for message of type '<ConfigDeleteModbus-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# config_delete_modbus ~%# It is a service to delete the Modbus I / O signal information registered ~%# in advance in the robot controller ~%#____________________________________________________________________________________________~%~%string      name       # modbus signal symbol ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigDeleteModbus-request)))
  "Returns full string definition for message of type 'ConfigDeleteModbus-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# config_delete_modbus ~%# It is a service to delete the Modbus I / O signal information registered ~%# in advance in the robot controller ~%#____________________________________________________________________________________________~%~%string      name       # modbus signal symbol ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigDeleteModbus-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigDeleteModbus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigDeleteModbus-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude ConfigDeleteModbus-response.msg.html

(cl:defclass <ConfigDeleteModbus-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ConfigDeleteModbus-response (<ConfigDeleteModbus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigDeleteModbus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigDeleteModbus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ConfigDeleteModbus-response> is deprecated: use dsr_msgs-srv:ConfigDeleteModbus-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ConfigDeleteModbus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigDeleteModbus-response>) ostream)
  "Serializes a message object of type '<ConfigDeleteModbus-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigDeleteModbus-response>) istream)
  "Deserializes a message object of type '<ConfigDeleteModbus-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigDeleteModbus-response>)))
  "Returns string type for a service object of type '<ConfigDeleteModbus-response>"
  "dsr_msgs/ConfigDeleteModbusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigDeleteModbus-response)))
  "Returns string type for a service object of type 'ConfigDeleteModbus-response"
  "dsr_msgs/ConfigDeleteModbusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigDeleteModbus-response>)))
  "Returns md5sum for a message object of type '<ConfigDeleteModbus-response>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigDeleteModbus-response)))
  "Returns md5sum for a message object of type 'ConfigDeleteModbus-response"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigDeleteModbus-response>)))
  "Returns full string definition for message of type '<ConfigDeleteModbus-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigDeleteModbus-response)))
  "Returns full string definition for message of type 'ConfigDeleteModbus-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigDeleteModbus-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigDeleteModbus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigDeleteModbus-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConfigDeleteModbus)))
  'ConfigDeleteModbus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConfigDeleteModbus)))
  'ConfigDeleteModbus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigDeleteModbus)))
  "Returns string type for a service object of type '<ConfigDeleteModbus>"
  "dsr_msgs/ConfigDeleteModbus")