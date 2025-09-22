; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude ConfigCreateModbus-request.msg.html

(cl:defclass <ConfigCreateModbus-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (ip
    :reader ip
    :initarg :ip
    :type cl:string
    :initform "")
   (port
    :reader port
    :initarg :port
    :type cl:integer
    :initform 0)
   (reg_type
    :reader reg_type
    :initarg :reg_type
    :type cl:fixnum
    :initform 0)
   (index
    :reader index
    :initarg :index
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0)
   (slave_id
    :reader slave_id
    :initarg :slave_id
    :type cl:integer
    :initform 0))
)

(cl:defclass ConfigCreateModbus-request (<ConfigCreateModbus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigCreateModbus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigCreateModbus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ConfigCreateModbus-request> is deprecated: use dsr_msgs-srv:ConfigCreateModbus-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ConfigCreateModbus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:name-val is deprecated.  Use dsr_msgs-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'ip-val :lambda-list '(m))
(cl:defmethod ip-val ((m <ConfigCreateModbus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ip-val is deprecated.  Use dsr_msgs-srv:ip instead.")
  (ip m))

(cl:ensure-generic-function 'port-val :lambda-list '(m))
(cl:defmethod port-val ((m <ConfigCreateModbus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:port-val is deprecated.  Use dsr_msgs-srv:port instead.")
  (port m))

(cl:ensure-generic-function 'reg_type-val :lambda-list '(m))
(cl:defmethod reg_type-val ((m <ConfigCreateModbus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:reg_type-val is deprecated.  Use dsr_msgs-srv:reg_type instead.")
  (reg_type m))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <ConfigCreateModbus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:index-val is deprecated.  Use dsr_msgs-srv:index instead.")
  (index m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <ConfigCreateModbus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:value-val is deprecated.  Use dsr_msgs-srv:value instead.")
  (value m))

(cl:ensure-generic-function 'slave_id-val :lambda-list '(m))
(cl:defmethod slave_id-val ((m <ConfigCreateModbus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:slave_id-val is deprecated.  Use dsr_msgs-srv:slave_id instead.")
  (slave_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigCreateModbus-request>) ostream)
  "Serializes a message object of type '<ConfigCreateModbus-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ip))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ip))
  (cl:let* ((signed (cl:slot-value msg 'port)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'reg_type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'slave_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigCreateModbus-request>) istream)
  "Deserializes a message object of type '<ConfigCreateModbus-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ip) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ip) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'port) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reg_type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'slave_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigCreateModbus-request>)))
  "Returns string type for a service object of type '<ConfigCreateModbus-request>"
  "dsr_msgs/ConfigCreateModbusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigCreateModbus-request)))
  "Returns string type for a service object of type 'ConfigCreateModbus-request"
  "dsr_msgs/ConfigCreateModbusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigCreateModbus-request>)))
  "Returns md5sum for a message object of type '<ConfigCreateModbus-request>"
  "21b56fd5fb77f7fccfc8eb6d995cd64a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigCreateModbus-request)))
  "Returns md5sum for a message object of type 'ConfigCreateModbus-request"
  "21b56fd5fb77f7fccfc8eb6d995cd64a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigCreateModbus-request>)))
  "Returns full string definition for message of type '<ConfigCreateModbus-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# config_create_modbus  ~%# This service registers the Modbus signal. ~%#____________________________________________________________________________________________~%~%string      name       # modbus signal symbol ~%string      ip         # external device ip~%int32       port       # external device port     ~%int8        reg_type   # <MODBUS_REGISTER_TYPE>(0: discrete input, 1: coil, 2: input register, 3: holding register)~%int8        index      # modbus signal index(0 ~~ 9999)~%int8        value      # modbus singla value(unsigned value ; 0 ~~ 65535)~%int32       slave_id   # Slave ID of the ModbusTCP(0: Broadcase address or 1-247 or 255: Default value for ModbusTCP) ~%                       # <slave_id is only available in M2.40 or later versions>  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigCreateModbus-request)))
  "Returns full string definition for message of type 'ConfigCreateModbus-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# config_create_modbus  ~%# This service registers the Modbus signal. ~%#____________________________________________________________________________________________~%~%string      name       # modbus signal symbol ~%string      ip         # external device ip~%int32       port       # external device port     ~%int8        reg_type   # <MODBUS_REGISTER_TYPE>(0: discrete input, 1: coil, 2: input register, 3: holding register)~%int8        index      # modbus signal index(0 ~~ 9999)~%int8        value      # modbus singla value(unsigned value ; 0 ~~ 65535)~%int32       slave_id   # Slave ID of the ModbusTCP(0: Broadcase address or 1-247 or 255: Default value for ModbusTCP) ~%                       # <slave_id is only available in M2.40 or later versions>  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigCreateModbus-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'ip))
     4
     1
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigCreateModbus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigCreateModbus-request
    (cl:cons ':name (name msg))
    (cl:cons ':ip (ip msg))
    (cl:cons ':port (port msg))
    (cl:cons ':reg_type (reg_type msg))
    (cl:cons ':index (index msg))
    (cl:cons ':value (value msg))
    (cl:cons ':slave_id (slave_id msg))
))
;//! \htmlinclude ConfigCreateModbus-response.msg.html

(cl:defclass <ConfigCreateModbus-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ConfigCreateModbus-response (<ConfigCreateModbus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigCreateModbus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigCreateModbus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ConfigCreateModbus-response> is deprecated: use dsr_msgs-srv:ConfigCreateModbus-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ConfigCreateModbus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigCreateModbus-response>) ostream)
  "Serializes a message object of type '<ConfigCreateModbus-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigCreateModbus-response>) istream)
  "Deserializes a message object of type '<ConfigCreateModbus-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigCreateModbus-response>)))
  "Returns string type for a service object of type '<ConfigCreateModbus-response>"
  "dsr_msgs/ConfigCreateModbusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigCreateModbus-response)))
  "Returns string type for a service object of type 'ConfigCreateModbus-response"
  "dsr_msgs/ConfigCreateModbusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigCreateModbus-response>)))
  "Returns md5sum for a message object of type '<ConfigCreateModbus-response>"
  "21b56fd5fb77f7fccfc8eb6d995cd64a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigCreateModbus-response)))
  "Returns md5sum for a message object of type 'ConfigCreateModbus-response"
  "21b56fd5fb77f7fccfc8eb6d995cd64a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigCreateModbus-response>)))
  "Returns full string definition for message of type '<ConfigCreateModbus-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigCreateModbus-response)))
  "Returns full string definition for message of type 'ConfigCreateModbus-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigCreateModbus-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigCreateModbus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigCreateModbus-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConfigCreateModbus)))
  'ConfigCreateModbus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConfigCreateModbus)))
  'ConfigCreateModbus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigCreateModbus)))
  "Returns string type for a service object of type '<ConfigCreateModbus>"
  "dsr_msgs/ConfigCreateModbus")