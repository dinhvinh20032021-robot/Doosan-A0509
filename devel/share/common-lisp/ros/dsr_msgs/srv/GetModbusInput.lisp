; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetModbusInput-request.msg.html

(cl:defclass <GetModbusInput-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass GetModbusInput-request (<GetModbusInput-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModbusInput-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModbusInput-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetModbusInput-request> is deprecated: use dsr_msgs-srv:GetModbusInput-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <GetModbusInput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:name-val is deprecated.  Use dsr_msgs-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModbusInput-request>) ostream)
  "Serializes a message object of type '<GetModbusInput-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModbusInput-request>) istream)
  "Deserializes a message object of type '<GetModbusInput-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModbusInput-request>)))
  "Returns string type for a service object of type '<GetModbusInput-request>"
  "dsr_msgs/GetModbusInputRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModbusInput-request)))
  "Returns string type for a service object of type 'GetModbusInput-request"
  "dsr_msgs/GetModbusInputRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModbusInput-request>)))
  "Returns md5sum for a message object of type '<GetModbusInput-request>"
  "a1db3dc69bfd7ece556367d29fe90139")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModbusInput-request)))
  "Returns md5sum for a message object of type 'GetModbusInput-request"
  "a1db3dc69bfd7ece556367d29fe90139")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModbusInput-request>)))
  "Returns full string definition for message of type '<GetModbusInput-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_modbus_input  ~%# This service reads the signal from the Modbus system.~%#____________________________________________________________________________________________~%~%string      name    # modbus signal symbol~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModbusInput-request)))
  "Returns full string definition for message of type 'GetModbusInput-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_modbus_input  ~%# This service reads the signal from the Modbus system.~%#____________________________________________________________________________________________~%~%string      name    # modbus signal symbol~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModbusInput-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModbusInput-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModbusInput-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude GetModbusInput-response.msg.html

(cl:defclass <GetModbusInput-response> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetModbusInput-response (<GetModbusInput-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModbusInput-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModbusInput-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetModbusInput-response> is deprecated: use dsr_msgs-srv:GetModbusInput-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <GetModbusInput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:value-val is deprecated.  Use dsr_msgs-srv:value instead.")
  (value m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetModbusInput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModbusInput-response>) ostream)
  "Serializes a message object of type '<GetModbusInput-response>"
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModbusInput-response>) istream)
  "Deserializes a message object of type '<GetModbusInput-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModbusInput-response>)))
  "Returns string type for a service object of type '<GetModbusInput-response>"
  "dsr_msgs/GetModbusInputResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModbusInput-response)))
  "Returns string type for a service object of type 'GetModbusInput-response"
  "dsr_msgs/GetModbusInputResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModbusInput-response>)))
  "Returns md5sum for a message object of type '<GetModbusInput-response>"
  "a1db3dc69bfd7ece556367d29fe90139")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModbusInput-response)))
  "Returns md5sum for a message object of type 'GetModbusInput-response"
  "a1db3dc69bfd7ece556367d29fe90139")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModbusInput-response>)))
  "Returns full string definition for message of type '<GetModbusInput-response>"
  (cl:format cl:nil "int32       value    # modbus signal value~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModbusInput-response)))
  "Returns full string definition for message of type 'GetModbusInput-response"
  (cl:format cl:nil "int32       value    # modbus signal value~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModbusInput-response>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModbusInput-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModbusInput-response
    (cl:cons ':value (value msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetModbusInput)))
  'GetModbusInput-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetModbusInput)))
  'GetModbusInput-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModbusInput)))
  "Returns string type for a service object of type '<GetModbusInput>"
  "dsr_msgs/GetModbusInput")