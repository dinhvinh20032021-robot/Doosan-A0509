; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-msg)


;//! \htmlinclude LogAlarm.msg.html

(cl:defclass <LogAlarm> (roslisp-msg-protocol:ros-message)
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
   (index
    :reader index
    :initarg :index
    :type cl:integer
    :initform 0)
   (param
    :reader param
    :initarg :param
    :type (cl:vector cl:string)
   :initform (cl:make-array 3 :element-type 'cl:string :initial-element "")))
)

(cl:defclass LogAlarm (<LogAlarm>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LogAlarm>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LogAlarm)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-msg:<LogAlarm> is deprecated: use dsr_msgs-msg:LogAlarm instead.")))

(cl:ensure-generic-function 'level-val :lambda-list '(m))
(cl:defmethod level-val ((m <LogAlarm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:level-val is deprecated.  Use dsr_msgs-msg:level instead.")
  (level m))

(cl:ensure-generic-function 'group-val :lambda-list '(m))
(cl:defmethod group-val ((m <LogAlarm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:group-val is deprecated.  Use dsr_msgs-msg:group instead.")
  (group m))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <LogAlarm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:index-val is deprecated.  Use dsr_msgs-msg:index instead.")
  (index m))

(cl:ensure-generic-function 'param-val :lambda-list '(m))
(cl:defmethod param-val ((m <LogAlarm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:param-val is deprecated.  Use dsr_msgs-msg:param instead.")
  (param m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LogAlarm>) ostream)
  "Serializes a message object of type '<LogAlarm>"
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
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'param))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LogAlarm>) istream)
  "Deserializes a message object of type '<LogAlarm>"
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
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:setf (cl:slot-value msg 'param) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'param)))
    (cl:dotimes (i 3)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LogAlarm>)))
  "Returns string type for a message object of type '<LogAlarm>"
  "dsr_msgs/LogAlarm")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LogAlarm)))
  "Returns string type for a message object of type 'LogAlarm"
  "dsr_msgs/LogAlarm")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LogAlarm>)))
  "Returns md5sum for a message object of type '<LogAlarm>"
  "c127c7c1149264259595eb8c3ff9972c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LogAlarm)))
  "Returns md5sum for a message object of type 'LogAlarm"
  "c127c7c1149264259595eb8c3ff9972c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LogAlarm>)))
  "Returns full string definition for message of type '<LogAlarm>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# log of alarm~%#____________________________________________________________________________________________~%~%int32         level~%int32         group~%int32         index~%string[3]     param~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LogAlarm)))
  "Returns full string definition for message of type 'LogAlarm"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# log of alarm~%#____________________________________________________________________________________________~%~%int32         level~%int32         group~%int32         index~%string[3]     param~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LogAlarm>))
  (cl:+ 0
     4
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'param) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LogAlarm>))
  "Converts a ROS message object to a list"
  (cl:list 'LogAlarm
    (cl:cons ':level (level msg))
    (cl:cons ':group (group msg))
    (cl:cons ':index (index msg))
    (cl:cons ':param (param msg))
))
