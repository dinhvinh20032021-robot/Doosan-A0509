; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetDesiredVelx-request.msg.html

(cl:defclass <GetDesiredVelx-request> (roslisp-msg-protocol:ros-message)
  ((ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetDesiredVelx-request (<GetDesiredVelx-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDesiredVelx-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDesiredVelx-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetDesiredVelx-request> is deprecated: use dsr_msgs-srv:GetDesiredVelx-request instead.")))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <GetDesiredVelx-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDesiredVelx-request>) ostream)
  "Serializes a message object of type '<GetDesiredVelx-request>"
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDesiredVelx-request>) istream)
  "Deserializes a message object of type '<GetDesiredVelx-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDesiredVelx-request>)))
  "Returns string type for a service object of type '<GetDesiredVelx-request>"
  "dsr_msgs/GetDesiredVelxRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDesiredVelx-request)))
  "Returns string type for a service object of type 'GetDesiredVelx-request"
  "dsr_msgs/GetDesiredVelxRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDesiredVelx-request>)))
  "Returns md5sum for a message object of type '<GetDesiredVelx-request>"
  "1ccbebbf66c48f74afb258f026a6daff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDesiredVelx-request)))
  "Returns md5sum for a message object of type 'GetDesiredVelx-request"
  "1ccbebbf66c48f74afb258f026a6daff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDesiredVelx-request>)))
  "Returns full string definition for message of type '<GetDesiredVelx-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_desired_velx(ref)  ~%#____________________________________________________________________________________________~%# This service returns the target velocity of the current tool based on the ref coordinate. ~%# It cannot be used in the movej, movejx, or movesj service.~%~%int8       ref               # DR_BASE(0), DR_WORLD(2)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDesiredVelx-request)))
  "Returns full string definition for message of type 'GetDesiredVelx-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_desired_velx(ref)  ~%#____________________________________________________________________________________________~%# This service returns the target velocity of the current tool based on the ref coordinate. ~%# It cannot be used in the movej, movejx, or movesj service.~%~%int8       ref               # DR_BASE(0), DR_WORLD(2)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDesiredVelx-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDesiredVelx-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDesiredVelx-request
    (cl:cons ':ref (ref msg))
))
;//! \htmlinclude GetDesiredVelx-response.msg.html

(cl:defclass <GetDesiredVelx-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetDesiredVelx-response (<GetDesiredVelx-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDesiredVelx-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDesiredVelx-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetDesiredVelx-response> is deprecated: use dsr_msgs-srv:GetDesiredVelx-response instead.")))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <GetDesiredVelx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:vel-val is deprecated.  Use dsr_msgs-srv:vel instead.")
  (vel m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetDesiredVelx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDesiredVelx-response>) ostream)
  "Serializes a message object of type '<GetDesiredVelx-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDesiredVelx-response>) istream)
  "Deserializes a message object of type '<GetDesiredVelx-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDesiredVelx-response>)))
  "Returns string type for a service object of type '<GetDesiredVelx-response>"
  "dsr_msgs/GetDesiredVelxResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDesiredVelx-response)))
  "Returns string type for a service object of type 'GetDesiredVelx-response"
  "dsr_msgs/GetDesiredVelxResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDesiredVelx-response>)))
  "Returns md5sum for a message object of type '<GetDesiredVelx-response>"
  "1ccbebbf66c48f74afb258f026a6daff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDesiredVelx-response)))
  "Returns md5sum for a message object of type 'GetDesiredVelx-response"
  "1ccbebbf66c48f74afb258f026a6daff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDesiredVelx-response>)))
  "Returns full string definition for message of type '<GetDesiredVelx-response>"
  (cl:format cl:nil "float64[6] vel               # Tool velocity~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDesiredVelx-response)))
  "Returns full string definition for message of type 'GetDesiredVelx-response"
  (cl:format cl:nil "float64[6] vel               # Tool velocity~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDesiredVelx-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'vel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDesiredVelx-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDesiredVelx-response
    (cl:cons ':vel (vel msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetDesiredVelx)))
  'GetDesiredVelx-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetDesiredVelx)))
  'GetDesiredVelx-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDesiredVelx)))
  "Returns string type for a service object of type '<GetDesiredVelx>"
  "dsr_msgs/GetDesiredVelx")