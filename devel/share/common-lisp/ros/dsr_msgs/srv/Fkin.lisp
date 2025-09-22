; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude Fkin-request.msg.html

(cl:defclass <Fkin-request> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Fkin-request (<Fkin-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Fkin-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Fkin-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<Fkin-request> is deprecated: use dsr_msgs-srv:Fkin-request instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <Fkin-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:pos-val is deprecated.  Use dsr_msgs-srv:pos instead.")
  (pos m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <Fkin-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Fkin-request>) ostream)
  "Serializes a message object of type '<Fkin-request>"
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
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Fkin-request>) istream)
  "Deserializes a message object of type '<Fkin-request>"
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Fkin-request>)))
  "Returns string type for a service object of type '<Fkin-request>"
  "dsr_msgs/FkinRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Fkin-request)))
  "Returns string type for a service object of type 'Fkin-request"
  "dsr_msgs/FkinRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Fkin-request>)))
  "Returns md5sum for a message object of type '<Fkin-request>"
  "26c14bd0ea35ff7d87f4f121a2ae40ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Fkin-request)))
  "Returns md5sum for a message object of type 'Fkin-request"
  "26c14bd0ea35ff7d87f4f121a2ae40ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Fkin-request>)))
  "Returns full string definition for message of type '<Fkin-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# fkin  ~%#____________________________________________________________________________________________~%~%float64[6] pos               # joint pos(posj)  ~%int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Fkin-request)))
  "Returns full string definition for message of type 'Fkin-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# fkin  ~%#____________________________________________________________________________________________~%~%float64[6] pos               # joint pos(posj)  ~%int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Fkin-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Fkin-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Fkin-request
    (cl:cons ':pos (pos msg))
    (cl:cons ':ref (ref msg))
))
;//! \htmlinclude Fkin-response.msg.html

(cl:defclass <Fkin-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Fkin-response (<Fkin-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Fkin-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Fkin-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<Fkin-response> is deprecated: use dsr_msgs-srv:Fkin-response instead.")))

(cl:ensure-generic-function 'conv_posx-val :lambda-list '(m))
(cl:defmethod conv_posx-val ((m <Fkin-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:conv_posx-val is deprecated.  Use dsr_msgs-srv:conv_posx instead.")
  (conv_posx m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Fkin-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Fkin-response>) ostream)
  "Serializes a message object of type '<Fkin-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Fkin-response>) istream)
  "Deserializes a message object of type '<Fkin-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Fkin-response>)))
  "Returns string type for a service object of type '<Fkin-response>"
  "dsr_msgs/FkinResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Fkin-response)))
  "Returns string type for a service object of type 'Fkin-response"
  "dsr_msgs/FkinResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Fkin-response>)))
  "Returns md5sum for a message object of type '<Fkin-response>"
  "26c14bd0ea35ff7d87f4f121a2ae40ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Fkin-response)))
  "Returns md5sum for a message object of type 'Fkin-response"
  "26c14bd0ea35ff7d87f4f121a2ae40ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Fkin-response>)))
  "Returns full string definition for message of type '<Fkin-response>"
  (cl:format cl:nil "float64[6] conv_posx         # task pos(posx)~%bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Fkin-response)))
  "Returns full string definition for message of type 'Fkin-response"
  (cl:format cl:nil "float64[6] conv_posx         # task pos(posx)~%bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Fkin-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'conv_posx) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Fkin-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Fkin-response
    (cl:cons ':conv_posx (conv_posx msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Fkin)))
  'Fkin-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Fkin)))
  'Fkin-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Fkin)))
  "Returns string type for a service object of type '<Fkin>"
  "dsr_msgs/Fkin")