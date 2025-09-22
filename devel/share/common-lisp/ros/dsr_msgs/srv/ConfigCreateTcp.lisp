; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude ConfigCreateTcp-request.msg.html

(cl:defclass <ConfigCreateTcp-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (pos
    :reader pos
    :initarg :pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ConfigCreateTcp-request (<ConfigCreateTcp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigCreateTcp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigCreateTcp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ConfigCreateTcp-request> is deprecated: use dsr_msgs-srv:ConfigCreateTcp-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ConfigCreateTcp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:name-val is deprecated.  Use dsr_msgs-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <ConfigCreateTcp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:pos-val is deprecated.  Use dsr_msgs-srv:pos instead.")
  (pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigCreateTcp-request>) ostream)
  "Serializes a message object of type '<ConfigCreateTcp-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'pos))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigCreateTcp-request>) istream)
  "Deserializes a message object of type '<ConfigCreateTcp-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:setf (cl:slot-value msg 'pos) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'pos)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigCreateTcp-request>)))
  "Returns string type for a service object of type '<ConfigCreateTcp-request>"
  "dsr_msgs/ConfigCreateTcpRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigCreateTcp-request)))
  "Returns string type for a service object of type 'ConfigCreateTcp-request"
  "dsr_msgs/ConfigCreateTcpRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigCreateTcp-request>)))
  "Returns md5sum for a message object of type '<ConfigCreateTcp-request>"
  "0c74c9733ff924f346cde676e47aa534")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigCreateTcp-request)))
  "Returns md5sum for a message object of type 'ConfigCreateTcp-request"
  "0c74c9733ff924f346cde676e47aa534")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigCreateTcp-request>)))
  "Returns full string definition for message of type '<ConfigCreateTcp-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# config_create_tcp  ~%# It is a service for registering and using robot TCP information in advance for safety~%#____________________________________________________________________________________________~%~%string          name         # tcp name ~%float64[6]      pos          # coordinates of the TCP ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigCreateTcp-request)))
  "Returns full string definition for message of type 'ConfigCreateTcp-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# config_create_tcp  ~%# It is a service for registering and using robot TCP information in advance for safety~%#____________________________________________________________________________________________~%~%string          name         # tcp name ~%float64[6]      pos          # coordinates of the TCP ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigCreateTcp-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigCreateTcp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigCreateTcp-request
    (cl:cons ':name (name msg))
    (cl:cons ':pos (pos msg))
))
;//! \htmlinclude ConfigCreateTcp-response.msg.html

(cl:defclass <ConfigCreateTcp-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ConfigCreateTcp-response (<ConfigCreateTcp-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigCreateTcp-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigCreateTcp-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ConfigCreateTcp-response> is deprecated: use dsr_msgs-srv:ConfigCreateTcp-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ConfigCreateTcp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigCreateTcp-response>) ostream)
  "Serializes a message object of type '<ConfigCreateTcp-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigCreateTcp-response>) istream)
  "Deserializes a message object of type '<ConfigCreateTcp-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigCreateTcp-response>)))
  "Returns string type for a service object of type '<ConfigCreateTcp-response>"
  "dsr_msgs/ConfigCreateTcpResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigCreateTcp-response)))
  "Returns string type for a service object of type 'ConfigCreateTcp-response"
  "dsr_msgs/ConfigCreateTcpResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigCreateTcp-response>)))
  "Returns md5sum for a message object of type '<ConfigCreateTcp-response>"
  "0c74c9733ff924f346cde676e47aa534")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigCreateTcp-response)))
  "Returns md5sum for a message object of type 'ConfigCreateTcp-response"
  "0c74c9733ff924f346cde676e47aa534")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigCreateTcp-response>)))
  "Returns full string definition for message of type '<ConfigCreateTcp-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigCreateTcp-response)))
  "Returns full string definition for message of type 'ConfigCreateTcp-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigCreateTcp-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigCreateTcp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigCreateTcp-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConfigCreateTcp)))
  'ConfigCreateTcp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConfigCreateTcp)))
  'ConfigCreateTcp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigCreateTcp)))
  "Returns string type for a service object of type '<ConfigCreateTcp>"
  "dsr_msgs/ConfigCreateTcp")