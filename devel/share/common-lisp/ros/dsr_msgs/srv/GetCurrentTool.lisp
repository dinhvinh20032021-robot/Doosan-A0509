; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetCurrentTool-request.msg.html

(cl:defclass <GetCurrentTool-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetCurrentTool-request (<GetCurrentTool-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentTool-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentTool-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCurrentTool-request> is deprecated: use dsr_msgs-srv:GetCurrentTool-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentTool-request>) ostream)
  "Serializes a message object of type '<GetCurrentTool-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentTool-request>) istream)
  "Deserializes a message object of type '<GetCurrentTool-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentTool-request>)))
  "Returns string type for a service object of type '<GetCurrentTool-request>"
  "dsr_msgs/GetCurrentToolRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentTool-request)))
  "Returns string type for a service object of type 'GetCurrentTool-request"
  "dsr_msgs/GetCurrentToolRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentTool-request>)))
  "Returns md5sum for a message object of type '<GetCurrentTool-request>"
  "4ed9e7e2e2815d295b7f655584191cad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentTool-request)))
  "Returns md5sum for a message object of type 'GetCurrentTool-request"
  "4ed9e7e2e2815d295b7f655584191cad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentTool-request>)))
  "Returns full string definition for message of type '<GetCurrentTool-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_current_tool  ~%# It is a service to fetch the currently set tool information from the robot controller~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentTool-request)))
  "Returns full string definition for message of type 'GetCurrentTool-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_current_tool  ~%# It is a service to fetch the currently set tool information from the robot controller~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentTool-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentTool-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentTool-request
))
;//! \htmlinclude GetCurrentTool-response.msg.html

(cl:defclass <GetCurrentTool-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetCurrentTool-response (<GetCurrentTool-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentTool-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentTool-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCurrentTool-response> is deprecated: use dsr_msgs-srv:GetCurrentTool-response instead.")))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <GetCurrentTool-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:info-val is deprecated.  Use dsr_msgs-srv:info instead.")
  (info m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetCurrentTool-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentTool-response>) ostream)
  "Serializes a message object of type '<GetCurrentTool-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'info))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentTool-response>) istream)
  "Deserializes a message object of type '<GetCurrentTool-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentTool-response>)))
  "Returns string type for a service object of type '<GetCurrentTool-response>"
  "dsr_msgs/GetCurrentToolResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentTool-response)))
  "Returns string type for a service object of type 'GetCurrentTool-response"
  "dsr_msgs/GetCurrentToolResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentTool-response>)))
  "Returns md5sum for a message object of type '<GetCurrentTool-response>"
  "4ed9e7e2e2815d295b7f655584191cad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentTool-response)))
  "Returns md5sum for a message object of type 'GetCurrentTool-response"
  "4ed9e7e2e2815d295b7f655584191cad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentTool-response>)))
  "Returns full string definition for message of type '<GetCurrentTool-response>"
  (cl:format cl:nil "string         info # tool name~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentTool-response)))
  "Returns full string definition for message of type 'GetCurrentTool-response"
  (cl:format cl:nil "string         info # tool name~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentTool-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'info))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentTool-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentTool-response
    (cl:cons ':info (info msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCurrentTool)))
  'GetCurrentTool-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCurrentTool)))
  'GetCurrentTool-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentTool)))
  "Returns string type for a service object of type '<GetCurrentTool>"
  "dsr_msgs/GetCurrentTool")