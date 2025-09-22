; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude ConnectRTControl-request.msg.html

(cl:defclass <ConnectRTControl-request> (roslisp-msg-protocol:ros-message)
  ((ip_address
    :reader ip_address
    :initarg :ip_address
    :type cl:string
    :initform "")
   (port
    :reader port
    :initarg :port
    :type cl:integer
    :initform 0))
)

(cl:defclass ConnectRTControl-request (<ConnectRTControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConnectRTControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConnectRTControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ConnectRTControl-request> is deprecated: use dsr_msgs-srv:ConnectRTControl-request instead.")))

(cl:ensure-generic-function 'ip_address-val :lambda-list '(m))
(cl:defmethod ip_address-val ((m <ConnectRTControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ip_address-val is deprecated.  Use dsr_msgs-srv:ip_address instead.")
  (ip_address m))

(cl:ensure-generic-function 'port-val :lambda-list '(m))
(cl:defmethod port-val ((m <ConnectRTControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:port-val is deprecated.  Use dsr_msgs-srv:port instead.")
  (port m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConnectRTControl-request>) ostream)
  "Serializes a message object of type '<ConnectRTControl-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ip_address))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ip_address))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'port)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'port)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'port)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConnectRTControl-request>) istream)
  "Deserializes a message object of type '<ConnectRTControl-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ip_address) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ip_address) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'port)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'port)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'port)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConnectRTControl-request>)))
  "Returns string type for a service object of type '<ConnectRTControl-request>"
  "dsr_msgs/ConnectRTControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConnectRTControl-request)))
  "Returns string type for a service object of type 'ConnectRTControl-request"
  "dsr_msgs/ConnectRTControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConnectRTControl-request>)))
  "Returns md5sum for a message object of type '<ConnectRTControl-request>"
  "924e165a726577b8d52c2048a873475b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConnectRTControl-request)))
  "Returns md5sum for a message object of type 'ConnectRTControl-request"
  "924e165a726577b8d52c2048a873475b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConnectRTControl-request>)))
  "Returns full string definition for message of type '<ConnectRTControl-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# connect_rt_control~%#____________________________________________________________________________________________~%~%string     ip_address~%uint32     port~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConnectRTControl-request)))
  "Returns full string definition for message of type 'ConnectRTControl-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# connect_rt_control~%#____________________________________________________________________________________________~%~%string     ip_address~%uint32     port~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConnectRTControl-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'ip_address))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConnectRTControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConnectRTControl-request
    (cl:cons ':ip_address (ip_address msg))
    (cl:cons ':port (port msg))
))
;//! \htmlinclude ConnectRTControl-response.msg.html

(cl:defclass <ConnectRTControl-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ConnectRTControl-response (<ConnectRTControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConnectRTControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConnectRTControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ConnectRTControl-response> is deprecated: use dsr_msgs-srv:ConnectRTControl-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ConnectRTControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConnectRTControl-response>) ostream)
  "Serializes a message object of type '<ConnectRTControl-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConnectRTControl-response>) istream)
  "Deserializes a message object of type '<ConnectRTControl-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConnectRTControl-response>)))
  "Returns string type for a service object of type '<ConnectRTControl-response>"
  "dsr_msgs/ConnectRTControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConnectRTControl-response)))
  "Returns string type for a service object of type 'ConnectRTControl-response"
  "dsr_msgs/ConnectRTControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConnectRTControl-response>)))
  "Returns md5sum for a message object of type '<ConnectRTControl-response>"
  "924e165a726577b8d52c2048a873475b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConnectRTControl-response)))
  "Returns md5sum for a message object of type 'ConnectRTControl-response"
  "924e165a726577b8d52c2048a873475b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConnectRTControl-response>)))
  "Returns full string definition for message of type '<ConnectRTControl-response>"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConnectRTControl-response)))
  "Returns full string definition for message of type 'ConnectRTControl-response"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConnectRTControl-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConnectRTControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConnectRTControl-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConnectRTControl)))
  'ConnectRTControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConnectRTControl)))
  'ConnectRTControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConnectRTControl)))
  "Returns string type for a service object of type '<ConnectRTControl>"
  "dsr_msgs/ConnectRTControl")