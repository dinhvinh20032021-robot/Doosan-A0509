; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude DrlStart-request.msg.html

(cl:defclass <DrlStart-request> (roslisp-msg-protocol:ros-message)
  ((robotSystem
    :reader robotSystem
    :initarg :robotSystem
    :type cl:fixnum
    :initform 0)
   (code
    :reader code
    :initarg :code
    :type cl:string
    :initform ""))
)

(cl:defclass DrlStart-request (<DrlStart-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DrlStart-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DrlStart-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<DrlStart-request> is deprecated: use dsr_msgs-srv:DrlStart-request instead.")))

(cl:ensure-generic-function 'robotSystem-val :lambda-list '(m))
(cl:defmethod robotSystem-val ((m <DrlStart-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:robotSystem-val is deprecated.  Use dsr_msgs-srv:robotSystem instead.")
  (robotSystem m))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <DrlStart-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:code-val is deprecated.  Use dsr_msgs-srv:code instead.")
  (code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DrlStart-request>) ostream)
  "Serializes a message object of type '<DrlStart-request>"
  (cl:let* ((signed (cl:slot-value msg 'robotSystem)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'code))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'code))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DrlStart-request>) istream)
  "Deserializes a message object of type '<DrlStart-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robotSystem) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'code) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'code) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DrlStart-request>)))
  "Returns string type for a service object of type '<DrlStart-request>"
  "dsr_msgs/DrlStartRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrlStart-request)))
  "Returns string type for a service object of type 'DrlStart-request"
  "dsr_msgs/DrlStartRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DrlStart-request>)))
  "Returns md5sum for a message object of type '<DrlStart-request>"
  "141a24be8b7e223a4678cb5eeb926bd6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DrlStart-request)))
  "Returns md5sum for a message object of type 'DrlStart-request"
  "141a24be8b7e223a4678cb5eeb926bd6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DrlStart-request>)))
  "Returns full string definition for message of type '<DrlStart-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# drl_script_run  ~%# This is a service to execute a program configured in the DRL language in the robot controller.~%#____________________________________________________________________________________________~%~%int8 robotSystem    # Robot System Mode 0 : Real, 1 : virtual~%string  code        # drl code       ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DrlStart-request)))
  "Returns full string definition for message of type 'DrlStart-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# drl_script_run  ~%# This is a service to execute a program configured in the DRL language in the robot controller.~%#____________________________________________________________________________________________~%~%int8 robotSystem    # Robot System Mode 0 : Real, 1 : virtual~%string  code        # drl code       ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DrlStart-request>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'code))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DrlStart-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DrlStart-request
    (cl:cons ':robotSystem (robotSystem msg))
    (cl:cons ':code (code msg))
))
;//! \htmlinclude DrlStart-response.msg.html

(cl:defclass <DrlStart-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DrlStart-response (<DrlStart-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DrlStart-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DrlStart-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<DrlStart-response> is deprecated: use dsr_msgs-srv:DrlStart-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DrlStart-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DrlStart-response>) ostream)
  "Serializes a message object of type '<DrlStart-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DrlStart-response>) istream)
  "Deserializes a message object of type '<DrlStart-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DrlStart-response>)))
  "Returns string type for a service object of type '<DrlStart-response>"
  "dsr_msgs/DrlStartResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrlStart-response)))
  "Returns string type for a service object of type 'DrlStart-response"
  "dsr_msgs/DrlStartResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DrlStart-response>)))
  "Returns md5sum for a message object of type '<DrlStart-response>"
  "141a24be8b7e223a4678cb5eeb926bd6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DrlStart-response)))
  "Returns md5sum for a message object of type 'DrlStart-response"
  "141a24be8b7e223a4678cb5eeb926bd6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DrlStart-response>)))
  "Returns full string definition for message of type '<DrlStart-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DrlStart-response)))
  "Returns full string definition for message of type 'DrlStart-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DrlStart-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DrlStart-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DrlStart-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DrlStart)))
  'DrlStart-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DrlStart)))
  'DrlStart-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrlStart)))
  "Returns string type for a service object of type '<DrlStart>"
  "dsr_msgs/DrlStart")