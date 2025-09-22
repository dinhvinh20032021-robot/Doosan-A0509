; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetCurrentTcp-request.msg.html

(cl:defclass <GetCurrentTcp-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetCurrentTcp-request (<GetCurrentTcp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentTcp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentTcp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCurrentTcp-request> is deprecated: use dsr_msgs-srv:GetCurrentTcp-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentTcp-request>) ostream)
  "Serializes a message object of type '<GetCurrentTcp-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentTcp-request>) istream)
  "Deserializes a message object of type '<GetCurrentTcp-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentTcp-request>)))
  "Returns string type for a service object of type '<GetCurrentTcp-request>"
  "dsr_msgs/GetCurrentTcpRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentTcp-request)))
  "Returns string type for a service object of type 'GetCurrentTcp-request"
  "dsr_msgs/GetCurrentTcpRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentTcp-request>)))
  "Returns md5sum for a message object of type '<GetCurrentTcp-request>"
  "4ed9e7e2e2815d295b7f655584191cad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentTcp-request)))
  "Returns md5sum for a message object of type 'GetCurrentTcp-request"
  "4ed9e7e2e2815d295b7f655584191cad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentTcp-request>)))
  "Returns full string definition for message of type '<GetCurrentTcp-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_current_tcp  ~%# It is the service to get the currently set TCP information from the robot controller~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentTcp-request)))
  "Returns full string definition for message of type 'GetCurrentTcp-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_current_tcp  ~%# It is the service to get the currently set TCP information from the robot controller~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentTcp-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentTcp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentTcp-request
))
;//! \htmlinclude GetCurrentTcp-response.msg.html

(cl:defclass <GetCurrentTcp-response> (roslisp-msg-protocol:ros-message)
  ((info
    :reader info
    :initarg :info
    :type cl:string
    :initform "")
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetCurrentTcp-response (<GetCurrentTcp-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentTcp-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentTcp-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCurrentTcp-response> is deprecated: use dsr_msgs-srv:GetCurrentTcp-response instead.")))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <GetCurrentTcp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:info-val is deprecated.  Use dsr_msgs-srv:info instead.")
  (info m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetCurrentTcp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentTcp-response>) ostream)
  "Serializes a message object of type '<GetCurrentTcp-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'info))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentTcp-response>) istream)
  "Deserializes a message object of type '<GetCurrentTcp-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'info) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'info) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentTcp-response>)))
  "Returns string type for a service object of type '<GetCurrentTcp-response>"
  "dsr_msgs/GetCurrentTcpResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentTcp-response)))
  "Returns string type for a service object of type 'GetCurrentTcp-response"
  "dsr_msgs/GetCurrentTcpResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentTcp-response>)))
  "Returns md5sum for a message object of type '<GetCurrentTcp-response>"
  "4ed9e7e2e2815d295b7f655584191cad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentTcp-response)))
  "Returns md5sum for a message object of type 'GetCurrentTcp-response"
  "4ed9e7e2e2815d295b7f655584191cad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentTcp-response>)))
  "Returns full string definition for message of type '<GetCurrentTcp-response>"
  (cl:format cl:nil "string         info # tcp name~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentTcp-response)))
  "Returns full string definition for message of type 'GetCurrentTcp-response"
  (cl:format cl:nil "string         info # tcp name~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentTcp-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'info))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentTcp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentTcp-response
    (cl:cons ':info (info msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCurrentTcp)))
  'GetCurrentTcp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCurrentTcp)))
  'GetCurrentTcp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentTcp)))
  "Returns string type for a service object of type '<GetCurrentTcp>"
  "dsr_msgs/GetCurrentTcp")