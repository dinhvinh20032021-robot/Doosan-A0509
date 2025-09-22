; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude ParallelAxis1-request.msg.html

(cl:defclass <ParallelAxis1-request> (roslisp-msg-protocol:ros-message)
  ((x1
    :reader x1
    :initarg :x1
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (x2
    :reader x2
    :initarg :x2
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (x3
    :reader x3
    :initarg :x3
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (axis
    :reader axis
    :initarg :axis
    :type cl:fixnum
    :initform 0)
   (ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ParallelAxis1-request (<ParallelAxis1-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ParallelAxis1-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ParallelAxis1-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ParallelAxis1-request> is deprecated: use dsr_msgs-srv:ParallelAxis1-request instead.")))

(cl:ensure-generic-function 'x1-val :lambda-list '(m))
(cl:defmethod x1-val ((m <ParallelAxis1-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:x1-val is deprecated.  Use dsr_msgs-srv:x1 instead.")
  (x1 m))

(cl:ensure-generic-function 'x2-val :lambda-list '(m))
(cl:defmethod x2-val ((m <ParallelAxis1-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:x2-val is deprecated.  Use dsr_msgs-srv:x2 instead.")
  (x2 m))

(cl:ensure-generic-function 'x3-val :lambda-list '(m))
(cl:defmethod x3-val ((m <ParallelAxis1-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:x3-val is deprecated.  Use dsr_msgs-srv:x3 instead.")
  (x3 m))

(cl:ensure-generic-function 'axis-val :lambda-list '(m))
(cl:defmethod axis-val ((m <ParallelAxis1-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:axis-val is deprecated.  Use dsr_msgs-srv:axis instead.")
  (axis m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <ParallelAxis1-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ParallelAxis1-request>) ostream)
  "Serializes a message object of type '<ParallelAxis1-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'x1))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'x2))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'x3))
  (cl:let* ((signed (cl:slot-value msg 'axis)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ParallelAxis1-request>) istream)
  "Deserializes a message object of type '<ParallelAxis1-request>"
  (cl:setf (cl:slot-value msg 'x1) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'x1)))
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
  (cl:setf (cl:slot-value msg 'x2) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'x2)))
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
  (cl:setf (cl:slot-value msg 'x3) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'x3)))
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'axis) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ParallelAxis1-request>)))
  "Returns string type for a service object of type '<ParallelAxis1-request>"
  "dsr_msgs/ParallelAxis1Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParallelAxis1-request)))
  "Returns string type for a service object of type 'ParallelAxis1-request"
  "dsr_msgs/ParallelAxis1Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ParallelAxis1-request>)))
  "Returns md5sum for a message object of type '<ParallelAxis1-request>"
  "f95b1583e6e04ac8d77cbe47de2af332")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ParallelAxis1-request)))
  "Returns md5sum for a message object of type 'ParallelAxis1-request"
  "f95b1583e6e04ac8d77cbe47de2af332")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ParallelAxis1-request>)))
  "Returns full string definition for message of type '<ParallelAxis1-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# parallel_axis(x1, x2, x3, axis, ref)  ~%#____________________________________________________________________________________________~%~%float64[6] x1                # task pos(posx)  ~%float64[6] x2                # task pos(posx)  ~%float64[6] x3                # task pos(posx)~%int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) ~%int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2), user coord(101~~200)~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ParallelAxis1-request)))
  "Returns full string definition for message of type 'ParallelAxis1-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# parallel_axis(x1, x2, x3, axis, ref)  ~%#____________________________________________________________________________________________~%~%float64[6] x1                # task pos(posx)  ~%float64[6] x2                # task pos(posx)  ~%float64[6] x3                # task pos(posx)~%int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) ~%int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2), user coord(101~~200)~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ParallelAxis1-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'x1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'x2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'x3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ParallelAxis1-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ParallelAxis1-request
    (cl:cons ':x1 (x1 msg))
    (cl:cons ':x2 (x2 msg))
    (cl:cons ':x3 (x3 msg))
    (cl:cons ':axis (axis msg))
    (cl:cons ':ref (ref msg))
))
;//! \htmlinclude ParallelAxis1-response.msg.html

(cl:defclass <ParallelAxis1-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ParallelAxis1-response (<ParallelAxis1-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ParallelAxis1-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ParallelAxis1-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ParallelAxis1-response> is deprecated: use dsr_msgs-srv:ParallelAxis1-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ParallelAxis1-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ParallelAxis1-response>) ostream)
  "Serializes a message object of type '<ParallelAxis1-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ParallelAxis1-response>) istream)
  "Deserializes a message object of type '<ParallelAxis1-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ParallelAxis1-response>)))
  "Returns string type for a service object of type '<ParallelAxis1-response>"
  "dsr_msgs/ParallelAxis1Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParallelAxis1-response)))
  "Returns string type for a service object of type 'ParallelAxis1-response"
  "dsr_msgs/ParallelAxis1Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ParallelAxis1-response>)))
  "Returns md5sum for a message object of type '<ParallelAxis1-response>"
  "f95b1583e6e04ac8d77cbe47de2af332")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ParallelAxis1-response)))
  "Returns md5sum for a message object of type 'ParallelAxis1-response"
  "f95b1583e6e04ac8d77cbe47de2af332")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ParallelAxis1-response>)))
  "Returns full string definition for message of type '<ParallelAxis1-response>"
  (cl:format cl:nil "bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ParallelAxis1-response)))
  "Returns full string definition for message of type 'ParallelAxis1-response"
  (cl:format cl:nil "bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ParallelAxis1-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ParallelAxis1-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ParallelAxis1-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ParallelAxis1)))
  'ParallelAxis1-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ParallelAxis1)))
  'ParallelAxis1-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParallelAxis1)))
  "Returns string type for a service object of type '<ParallelAxis1>"
  "dsr_msgs/ParallelAxis1")