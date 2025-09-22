; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude CalcCoord-request.msg.html

(cl:defclass <CalcCoord-request> (roslisp-msg-protocol:ros-message)
  ((input_pos_cnt
    :reader input_pos_cnt
    :initarg :input_pos_cnt
    :type cl:fixnum
    :initform 0)
   (x1
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
   (x4
    :reader x4
    :initarg :x4
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0)
   (mod
    :reader mod
    :initarg :mod
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CalcCoord-request (<CalcCoord-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalcCoord-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalcCoord-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<CalcCoord-request> is deprecated: use dsr_msgs-srv:CalcCoord-request instead.")))

(cl:ensure-generic-function 'input_pos_cnt-val :lambda-list '(m))
(cl:defmethod input_pos_cnt-val ((m <CalcCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:input_pos_cnt-val is deprecated.  Use dsr_msgs-srv:input_pos_cnt instead.")
  (input_pos_cnt m))

(cl:ensure-generic-function 'x1-val :lambda-list '(m))
(cl:defmethod x1-val ((m <CalcCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:x1-val is deprecated.  Use dsr_msgs-srv:x1 instead.")
  (x1 m))

(cl:ensure-generic-function 'x2-val :lambda-list '(m))
(cl:defmethod x2-val ((m <CalcCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:x2-val is deprecated.  Use dsr_msgs-srv:x2 instead.")
  (x2 m))

(cl:ensure-generic-function 'x3-val :lambda-list '(m))
(cl:defmethod x3-val ((m <CalcCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:x3-val is deprecated.  Use dsr_msgs-srv:x3 instead.")
  (x3 m))

(cl:ensure-generic-function 'x4-val :lambda-list '(m))
(cl:defmethod x4-val ((m <CalcCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:x4-val is deprecated.  Use dsr_msgs-srv:x4 instead.")
  (x4 m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <CalcCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))

(cl:ensure-generic-function 'mod-val :lambda-list '(m))
(cl:defmethod mod-val ((m <CalcCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:mod-val is deprecated.  Use dsr_msgs-srv:mod instead.")
  (mod m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalcCoord-request>) ostream)
  "Serializes a message object of type '<CalcCoord-request>"
  (cl:let* ((signed (cl:slot-value msg 'input_pos_cnt)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
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
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'x4))
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mod)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalcCoord-request>) istream)
  "Deserializes a message object of type '<CalcCoord-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'input_pos_cnt) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
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
  (cl:setf (cl:slot-value msg 'x4) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'x4)))
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
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mod) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalcCoord-request>)))
  "Returns string type for a service object of type '<CalcCoord-request>"
  "dsr_msgs/CalcCoordRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalcCoord-request)))
  "Returns string type for a service object of type 'CalcCoord-request"
  "dsr_msgs/CalcCoordRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalcCoord-request>)))
  "Returns md5sum for a message object of type '<CalcCoord-request>"
  "2001e63c6ceb43ba2ea8431cc72302b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalcCoord-request)))
  "Returns md5sum for a message object of type 'CalcCoord-request"
  "2001e63c6ceb43ba2ea8431cc72302b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalcCoord-request>)))
  "Returns full string definition for message of type '<CalcCoord-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# calc_coord   ~%#____________________________________________________________________________________________~%# This service is only available in M2.50 or later~%~%int8       input_pos_cnt     # input_pos_cnt~%float64[6] x1                # task pos(posx)  ~%float64[6] x2                # task pos(posx)  ~%float64[6] x3                # task pos(posx)~%float64[6] x4                # task pos(posx)~%int8       ref               # DR_BASE(0), DR_WORLD(2)~%int8       mod               # input mode(only valid when the number of input poses is 2)~%                             # 0: defining z-axis based on the current Tool-z direction~%                             # 1: defining z-axis based on the z direction of x1 ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalcCoord-request)))
  "Returns full string definition for message of type 'CalcCoord-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# calc_coord   ~%#____________________________________________________________________________________________~%# This service is only available in M2.50 or later~%~%int8       input_pos_cnt     # input_pos_cnt~%float64[6] x1                # task pos(posx)  ~%float64[6] x2                # task pos(posx)  ~%float64[6] x3                # task pos(posx)~%float64[6] x4                # task pos(posx)~%int8       ref               # DR_BASE(0), DR_WORLD(2)~%int8       mod               # input mode(only valid when the number of input poses is 2)~%                             # 0: defining z-axis based on the current Tool-z direction~%                             # 1: defining z-axis based on the z direction of x1 ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalcCoord-request>))
  (cl:+ 0
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'x1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'x2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'x3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'x4) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalcCoord-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CalcCoord-request
    (cl:cons ':input_pos_cnt (input_pos_cnt msg))
    (cl:cons ':x1 (x1 msg))
    (cl:cons ':x2 (x2 msg))
    (cl:cons ':x3 (x3 msg))
    (cl:cons ':x4 (x4 msg))
    (cl:cons ':ref (ref msg))
    (cl:cons ':mod (mod msg))
))
;//! \htmlinclude CalcCoord-response.msg.html

(cl:defclass <CalcCoord-response> (roslisp-msg-protocol:ros-message)
  ((conv_posx
    :reader conv_posx
    :initarg :conv_posx
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CalcCoord-response (<CalcCoord-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalcCoord-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalcCoord-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<CalcCoord-response> is deprecated: use dsr_msgs-srv:CalcCoord-response instead.")))

(cl:ensure-generic-function 'conv_posx-val :lambda-list '(m))
(cl:defmethod conv_posx-val ((m <CalcCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:conv_posx-val is deprecated.  Use dsr_msgs-srv:conv_posx instead.")
  (conv_posx m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <CalcCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalcCoord-response>) ostream)
  "Serializes a message object of type '<CalcCoord-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'conv_posx))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalcCoord-response>) istream)
  "Deserializes a message object of type '<CalcCoord-response>"
  (cl:setf (cl:slot-value msg 'conv_posx) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'conv_posx)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalcCoord-response>)))
  "Returns string type for a service object of type '<CalcCoord-response>"
  "dsr_msgs/CalcCoordResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalcCoord-response)))
  "Returns string type for a service object of type 'CalcCoord-response"
  "dsr_msgs/CalcCoordResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalcCoord-response>)))
  "Returns md5sum for a message object of type '<CalcCoord-response>"
  "2001e63c6ceb43ba2ea8431cc72302b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalcCoord-response)))
  "Returns md5sum for a message object of type 'CalcCoord-response"
  "2001e63c6ceb43ba2ea8431cc72302b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalcCoord-response>)))
  "Returns full string definition for message of type '<CalcCoord-response>"
  (cl:format cl:nil "float64[6] conv_posx         # task pos(posx) ~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalcCoord-response)))
  "Returns full string definition for message of type 'CalcCoord-response"
  (cl:format cl:nil "float64[6] conv_posx         # task pos(posx) ~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalcCoord-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'conv_posx) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalcCoord-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CalcCoord-response
    (cl:cons ':conv_posx (conv_posx msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CalcCoord)))
  'CalcCoord-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CalcCoord)))
  'CalcCoord-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalcCoord)))
  "Returns string type for a service object of type '<CalcCoord>"
  "dsr_msgs/CalcCoord")