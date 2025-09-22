; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-msg)


;//! \htmlinclude RobotError.msg.html

(cl:defclass <RobotError> (roslisp-msg-protocol:ros-message)
  ((level
    :reader level
    :initarg :level
    :type cl:integer
    :initform 0)
   (group
    :reader group
    :initarg :group
    :type cl:integer
    :initform 0)
   (code
    :reader code
    :initarg :code
    :type cl:integer
    :initform 0)
   (msg1
    :reader msg1
    :initarg :msg1
    :type cl:string
    :initform "")
   (msg2
    :reader msg2
    :initarg :msg2
    :type cl:string
    :initform "")
   (msg3
    :reader msg3
    :initarg :msg3
    :type cl:string
    :initform ""))
)

(cl:defclass RobotError (<RobotError>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotError>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotError)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-msg:<RobotError> is deprecated: use dsr_msgs-msg:RobotError instead.")))

(cl:ensure-generic-function 'level-val :lambda-list '(m))
(cl:defmethod level-val ((m <RobotError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:level-val is deprecated.  Use dsr_msgs-msg:level instead.")
  (level m))

(cl:ensure-generic-function 'group-val :lambda-list '(m))
(cl:defmethod group-val ((m <RobotError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:group-val is deprecated.  Use dsr_msgs-msg:group instead.")
  (group m))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <RobotError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:code-val is deprecated.  Use dsr_msgs-msg:code instead.")
  (code m))

(cl:ensure-generic-function 'msg1-val :lambda-list '(m))
(cl:defmethod msg1-val ((m <RobotError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:msg1-val is deprecated.  Use dsr_msgs-msg:msg1 instead.")
  (msg1 m))

(cl:ensure-generic-function 'msg2-val :lambda-list '(m))
(cl:defmethod msg2-val ((m <RobotError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:msg2-val is deprecated.  Use dsr_msgs-msg:msg2 instead.")
  (msg2 m))

(cl:ensure-generic-function 'msg3-val :lambda-list '(m))
(cl:defmethod msg3-val ((m <RobotError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:msg3-val is deprecated.  Use dsr_msgs-msg:msg3 instead.")
  (msg3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotError>) ostream)
  "Serializes a message object of type '<RobotError>"
  (cl:let* ((signed (cl:slot-value msg 'level)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'group)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg1))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg2))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg3))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotError>) istream)
  "Deserializes a message object of type '<RobotError>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'level) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'group) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'code) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg1) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg1) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg2) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg2) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg3) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg3) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotError>)))
  "Returns string type for a message object of type '<RobotError>"
  "dsr_msgs/RobotError")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotError)))
  "Returns string type for a message object of type 'RobotError"
  "dsr_msgs/RobotError")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotError>)))
  "Returns md5sum for a message object of type '<RobotError>"
  "2d9680ace6867f719d7e19bbc321e6da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotError)))
  "Returns md5sum for a message object of type 'RobotError"
  "2d9680ace6867f719d7e19bbc321e6da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotError>)))
  "Returns full string definition for message of type '<RobotError>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# [ robot error msg ] ~%#____________________________________________________________________________________________~%~%int32	    level   # INFO =1, WARN =2, ERROR =3 ~%int32	    group   # SYSTEM =1, MOTION =2, TP =3, INVERTER =4, SAFETY_CONTROLLER =5   ~%int32	    code    # error code ~%string      msg1    # error msg 1~%string      msg2    # error msg 2~%string      msg3    # error msg 3~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotError)))
  "Returns full string definition for message of type 'RobotError"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# [ robot error msg ] ~%#____________________________________________________________________________________________~%~%int32	    level   # INFO =1, WARN =2, ERROR =3 ~%int32	    group   # SYSTEM =1, MOTION =2, TP =3, INVERTER =4, SAFETY_CONTROLLER =5   ~%int32	    code    # error code ~%string      msg1    # error msg 1~%string      msg2    # error msg 2~%string      msg3    # error msg 3~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotError>))
  (cl:+ 0
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'msg1))
     4 (cl:length (cl:slot-value msg 'msg2))
     4 (cl:length (cl:slot-value msg 'msg3))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotError>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotError
    (cl:cons ':level (level msg))
    (cl:cons ':group (group msg))
    (cl:cons ':code (code msg))
    (cl:cons ':msg1 (msg1 msg))
    (cl:cons ':msg2 (msg2 msg))
    (cl:cons ':msg3 (msg3 msg))
))
