; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetCurrentVelx-request.msg.html

(cl:defclass <GetCurrentVelx-request> (roslisp-msg-protocol:ros-message)
  ((ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetCurrentVelx-request (<GetCurrentVelx-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentVelx-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentVelx-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCurrentVelx-request> is deprecated: use dsr_msgs-srv:GetCurrentVelx-request instead.")))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <GetCurrentVelx-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentVelx-request>) ostream)
  "Serializes a message object of type '<GetCurrentVelx-request>"
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentVelx-request>) istream)
  "Deserializes a message object of type '<GetCurrentVelx-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentVelx-request>)))
  "Returns string type for a service object of type '<GetCurrentVelx-request>"
  "dsr_msgs/GetCurrentVelxRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentVelx-request)))
  "Returns string type for a service object of type 'GetCurrentVelx-request"
  "dsr_msgs/GetCurrentVelxRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentVelx-request>)))
  "Returns md5sum for a message object of type '<GetCurrentVelx-request>"
  "1ccbebbf66c48f74afb258f026a6daff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentVelx-request)))
  "Returns md5sum for a message object of type 'GetCurrentVelx-request"
  "1ccbebbf66c48f74afb258f026a6daff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentVelx-request>)))
  "Returns full string definition for message of type '<GetCurrentVelx-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_current_velx(ref)  ~%#____________________________________________________________________________________________~%# This service returns the current tool velocity based on the ref coordinate.~%~%int8       ref               # DR_BASE(0), DR_WORLD(2)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentVelx-request)))
  "Returns full string definition for message of type 'GetCurrentVelx-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_current_velx(ref)  ~%#____________________________________________________________________________________________~%# This service returns the current tool velocity based on the ref coordinate.~%~%int8       ref               # DR_BASE(0), DR_WORLD(2)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentVelx-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentVelx-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentVelx-request
    (cl:cons ':ref (ref msg))
))
;//! \htmlinclude GetCurrentVelx-response.msg.html

(cl:defclass <GetCurrentVelx-response> (roslisp-msg-protocol:ros-message)
  ((vel
    :reader vel
    :initarg :vel
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetCurrentVelx-response (<GetCurrentVelx-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentVelx-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentVelx-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCurrentVelx-response> is deprecated: use dsr_msgs-srv:GetCurrentVelx-response instead.")))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <GetCurrentVelx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:vel-val is deprecated.  Use dsr_msgs-srv:vel instead.")
  (vel m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetCurrentVelx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentVelx-response>) ostream)
  "Serializes a message object of type '<GetCurrentVelx-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'vel))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentVelx-response>) istream)
  "Deserializes a message object of type '<GetCurrentVelx-response>"
  (cl:setf (cl:slot-value msg 'vel) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'vel)))
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
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentVelx-response>)))
  "Returns string type for a service object of type '<GetCurrentVelx-response>"
  "dsr_msgs/GetCurrentVelxResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentVelx-response)))
  "Returns string type for a service object of type 'GetCurrentVelx-response"
  "dsr_msgs/GetCurrentVelxResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentVelx-response>)))
  "Returns md5sum for a message object of type '<GetCurrentVelx-response>"
  "1ccbebbf66c48f74afb258f026a6daff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentVelx-response)))
  "Returns md5sum for a message object of type 'GetCurrentVelx-response"
  "1ccbebbf66c48f74afb258f026a6daff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentVelx-response>)))
  "Returns full string definition for message of type '<GetCurrentVelx-response>"
  (cl:format cl:nil "float64[6] vel               # Tool velocity~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentVelx-response)))
  "Returns full string definition for message of type 'GetCurrentVelx-response"
  (cl:format cl:nil "float64[6] vel               # Tool velocity~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentVelx-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'vel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentVelx-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentVelx-response
    (cl:cons ':vel (vel msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCurrentVelx)))
  'GetCurrentVelx-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCurrentVelx)))
  'GetCurrentVelx-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentVelx)))
  "Returns string type for a service object of type '<GetCurrentVelx>"
  "dsr_msgs/GetCurrentVelx")