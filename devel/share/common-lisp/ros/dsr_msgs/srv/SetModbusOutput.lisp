; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetModbusOutput-request.msg.html

(cl:defclass <SetModbusOutput-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass SetModbusOutput-request (<SetModbusOutput-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetModbusOutput-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetModbusOutput-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetModbusOutput-request> is deprecated: use dsr_msgs-srv:SetModbusOutput-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <SetModbusOutput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:name-val is deprecated.  Use dsr_msgs-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SetModbusOutput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:value-val is deprecated.  Use dsr_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetModbusOutput-request>) ostream)
  "Serializes a message object of type '<SetModbusOutput-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetModbusOutput-request>) istream)
  "Deserializes a message object of type '<SetModbusOutput-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetModbusOutput-request>)))
  "Returns string type for a service object of type '<SetModbusOutput-request>"
  "dsr_msgs/SetModbusOutputRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetModbusOutput-request)))
  "Returns string type for a service object of type 'SetModbusOutput-request"
  "dsr_msgs/SetModbusOutputRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetModbusOutput-request>)))
  "Returns md5sum for a message object of type '<SetModbusOutput-request>"
  "7e14dd01537ca042e8c020bcdd67b27a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetModbusOutput-request)))
  "Returns md5sum for a message object of type 'SetModbusOutput-request"
  "7e14dd01537ca042e8c020bcdd67b27a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetModbusOutput-request>)))
  "Returns full string definition for message of type '<SetModbusOutput-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_modbus_output  ~%# This service sends the signal to an external Modbus system. ~%#____________________________________________________________________________________________~%~%string      name     # modbus signal symbol~%int32       value    # modbus register value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetModbusOutput-request)))
  "Returns full string definition for message of type 'SetModbusOutput-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_modbus_output  ~%# This service sends the signal to an external Modbus system. ~%#____________________________________________________________________________________________~%~%string      name     # modbus signal symbol~%int32       value    # modbus register value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetModbusOutput-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetModbusOutput-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetModbusOutput-request
    (cl:cons ':name (name msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude SetModbusOutput-response.msg.html

(cl:defclass <SetModbusOutput-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetModbusOutput-response (<SetModbusOutput-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetModbusOutput-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetModbusOutput-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetModbusOutput-response> is deprecated: use dsr_msgs-srv:SetModbusOutput-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetModbusOutput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetModbusOutput-response>) ostream)
  "Serializes a message object of type '<SetModbusOutput-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetModbusOutput-response>) istream)
  "Deserializes a message object of type '<SetModbusOutput-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetModbusOutput-response>)))
  "Returns string type for a service object of type '<SetModbusOutput-response>"
  "dsr_msgs/SetModbusOutputResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetModbusOutput-response)))
  "Returns string type for a service object of type 'SetModbusOutput-response"
  "dsr_msgs/SetModbusOutputResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetModbusOutput-response>)))
  "Returns md5sum for a message object of type '<SetModbusOutput-response>"
  "7e14dd01537ca042e8c020bcdd67b27a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetModbusOutput-response)))
  "Returns md5sum for a message object of type 'SetModbusOutput-response"
  "7e14dd01537ca042e8c020bcdd67b27a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetModbusOutput-response>)))
  "Returns full string definition for message of type '<SetModbusOutput-response>"
  (cl:format cl:nil "bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetModbusOutput-response)))
  "Returns full string definition for message of type 'SetModbusOutput-response"
  (cl:format cl:nil "bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetModbusOutput-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetModbusOutput-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetModbusOutput-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetModbusOutput)))
  'SetModbusOutput-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetModbusOutput)))
  'SetModbusOutput-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetModbusOutput)))
  "Returns string type for a service object of type '<SetModbusOutput>"
  "dsr_msgs/SetModbusOutput")