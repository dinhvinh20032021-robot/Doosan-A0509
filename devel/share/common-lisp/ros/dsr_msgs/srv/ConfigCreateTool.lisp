; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude ConfigCreateTool-request.msg.html

(cl:defclass <ConfigCreateTool-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (weight
    :reader weight
    :initarg :weight
    :type cl:float
    :initform 0.0)
   (cog
    :reader cog
    :initarg :cog
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (inertia
    :reader inertia
    :initarg :inertia
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ConfigCreateTool-request (<ConfigCreateTool-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigCreateTool-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigCreateTool-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ConfigCreateTool-request> is deprecated: use dsr_msgs-srv:ConfigCreateTool-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ConfigCreateTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:name-val is deprecated.  Use dsr_msgs-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'weight-val :lambda-list '(m))
(cl:defmethod weight-val ((m <ConfigCreateTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:weight-val is deprecated.  Use dsr_msgs-srv:weight instead.")
  (weight m))

(cl:ensure-generic-function 'cog-val :lambda-list '(m))
(cl:defmethod cog-val ((m <ConfigCreateTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:cog-val is deprecated.  Use dsr_msgs-srv:cog instead.")
  (cog m))

(cl:ensure-generic-function 'inertia-val :lambda-list '(m))
(cl:defmethod inertia-val ((m <ConfigCreateTool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:inertia-val is deprecated.  Use dsr_msgs-srv:inertia instead.")
  (inertia m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigCreateTool-request>) ostream)
  "Serializes a message object of type '<ConfigCreateTool-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'weight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'cog))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'inertia))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigCreateTool-request>) istream)
  "Deserializes a message object of type '<ConfigCreateTool-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'weight) (roslisp-utils:decode-double-float-bits bits)))
  (cl:setf (cl:slot-value msg 'cog) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'cog)))
    (cl:dotimes (i 3)
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
  (cl:setf (cl:slot-value msg 'inertia) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'inertia)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigCreateTool-request>)))
  "Returns string type for a service object of type '<ConfigCreateTool-request>"
  "dsr_msgs/ConfigCreateToolRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigCreateTool-request)))
  "Returns string type for a service object of type 'ConfigCreateTool-request"
  "dsr_msgs/ConfigCreateToolRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigCreateTool-request>)))
  "Returns md5sum for a message object of type '<ConfigCreateTool-request>"
  "d748bfa923d2a13eaf013a95f67bd0ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigCreateTool-request)))
  "Returns md5sum for a message object of type 'ConfigCreateTool-request"
  "d748bfa923d2a13eaf013a95f67bd0ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigCreateTool-request>)))
  "Returns full string definition for message of type '<ConfigCreateTool-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# config_create_tool ~%# It is a service for registering and using robot Tool information in advance for safety ~%#____________________________________________________________________________________________~%~%string          name        # tool name ~%float64         weight      # tool weight ~%float64[3]      cog         # Center of gravity~%float64[6]      inertia     # tool inertia ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigCreateTool-request)))
  "Returns full string definition for message of type 'ConfigCreateTool-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# config_create_tool ~%# It is a service for registering and using robot Tool information in advance for safety ~%#____________________________________________________________________________________________~%~%string          name        # tool name ~%float64         weight      # tool weight ~%float64[3]      cog         # Center of gravity~%float64[6]      inertia     # tool inertia ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigCreateTool-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     8
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'cog) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'inertia) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigCreateTool-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigCreateTool-request
    (cl:cons ':name (name msg))
    (cl:cons ':weight (weight msg))
    (cl:cons ':cog (cog msg))
    (cl:cons ':inertia (inertia msg))
))
;//! \htmlinclude ConfigCreateTool-response.msg.html

(cl:defclass <ConfigCreateTool-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ConfigCreateTool-response (<ConfigCreateTool-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigCreateTool-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigCreateTool-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ConfigCreateTool-response> is deprecated: use dsr_msgs-srv:ConfigCreateTool-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ConfigCreateTool-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigCreateTool-response>) ostream)
  "Serializes a message object of type '<ConfigCreateTool-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigCreateTool-response>) istream)
  "Deserializes a message object of type '<ConfigCreateTool-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigCreateTool-response>)))
  "Returns string type for a service object of type '<ConfigCreateTool-response>"
  "dsr_msgs/ConfigCreateToolResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigCreateTool-response)))
  "Returns string type for a service object of type 'ConfigCreateTool-response"
  "dsr_msgs/ConfigCreateToolResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigCreateTool-response>)))
  "Returns md5sum for a message object of type '<ConfigCreateTool-response>"
  "d748bfa923d2a13eaf013a95f67bd0ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigCreateTool-response)))
  "Returns md5sum for a message object of type 'ConfigCreateTool-response"
  "d748bfa923d2a13eaf013a95f67bd0ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigCreateTool-response>)))
  "Returns full string definition for message of type '<ConfigCreateTool-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigCreateTool-response)))
  "Returns full string definition for message of type 'ConfigCreateTool-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigCreateTool-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigCreateTool-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigCreateTool-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConfigCreateTool)))
  'ConfigCreateTool-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConfigCreateTool)))
  'ConfigCreateTool-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigCreateTool)))
  "Returns string type for a service object of type '<ConfigCreateTool>"
  "dsr_msgs/ConfigCreateTool")