; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-msg)


;//! \htmlinclude ModbusState.msg.html

(cl:defclass <ModbusState> (roslisp-msg-protocol:ros-message)
  ((modbus_symbol
    :reader modbus_symbol
    :initarg :modbus_symbol
    :type cl:string
    :initform "")
   (modbus_value
    :reader modbus_value
    :initarg :modbus_value
    :type cl:integer
    :initform 0))
)

(cl:defclass ModbusState (<ModbusState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModbusState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModbusState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-msg:<ModbusState> is deprecated: use dsr_msgs-msg:ModbusState instead.")))

(cl:ensure-generic-function 'modbus_symbol-val :lambda-list '(m))
(cl:defmethod modbus_symbol-val ((m <ModbusState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:modbus_symbol-val is deprecated.  Use dsr_msgs-msg:modbus_symbol instead.")
  (modbus_symbol m))

(cl:ensure-generic-function 'modbus_value-val :lambda-list '(m))
(cl:defmethod modbus_value-val ((m <ModbusState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:modbus_value-val is deprecated.  Use dsr_msgs-msg:modbus_value instead.")
  (modbus_value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModbusState>) ostream)
  "Serializes a message object of type '<ModbusState>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'modbus_symbol))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'modbus_symbol))
  (cl:let* ((signed (cl:slot-value msg 'modbus_value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModbusState>) istream)
  "Deserializes a message object of type '<ModbusState>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'modbus_symbol) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'modbus_symbol) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'modbus_value) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModbusState>)))
  "Returns string type for a message object of type '<ModbusState>"
  "dsr_msgs/ModbusState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModbusState)))
  "Returns string type for a message object of type 'ModbusState"
  "dsr_msgs/ModbusState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModbusState>)))
  "Returns md5sum for a message object of type '<ModbusState>"
  "c941aa5cf7a235f21e3789f2d6e5305f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModbusState)))
  "Returns md5sum for a message object of type 'ModbusState"
  "c941aa5cf7a235f21e3789f2d6e5305f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModbusState>)))
  "Returns full string definition for message of type '<ModbusState>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%#Custom msg for RobotState.msg -- MAX_SIZE = 100~%#____________________________________________________________________________________________~%~%string  modbus_symbol    # Modbus Signal Name~%int32   modbus_value     # Modbus Register Value (Unsigned : 0 ~~ 65535)~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModbusState)))
  "Returns full string definition for message of type 'ModbusState"
  (cl:format cl:nil "#____________________________________________________________________________________________~%#Custom msg for RobotState.msg -- MAX_SIZE = 100~%#____________________________________________________________________________________________~%~%string  modbus_symbol    # Modbus Signal Name~%int32   modbus_value     # Modbus Register Value (Unsigned : 0 ~~ 65535)~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModbusState>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'modbus_symbol))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModbusState>))
  "Converts a ROS message object to a list"
  (cl:list 'ModbusState
    (cl:cons ':modbus_symbol (modbus_symbol msg))
    (cl:cons ':modbus_value (modbus_value msg))
))
