; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude Trans-request.msg.html

(cl:defclass <Trans-request> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (delta
    :reader delta
    :initarg :delta
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0)
   (ref_out
    :reader ref_out
    :initarg :ref_out
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Trans-request (<Trans-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Trans-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Trans-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<Trans-request> is deprecated: use dsr_msgs-srv:Trans-request instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <Trans-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:pos-val is deprecated.  Use dsr_msgs-srv:pos instead.")
  (pos m))

(cl:ensure-generic-function 'delta-val :lambda-list '(m))
(cl:defmethod delta-val ((m <Trans-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:delta-val is deprecated.  Use dsr_msgs-srv:delta instead.")
  (delta m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <Trans-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))

(cl:ensure-generic-function 'ref_out-val :lambda-list '(m))
(cl:defmethod ref_out-val ((m <Trans-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref_out-val is deprecated.  Use dsr_msgs-srv:ref_out instead.")
  (ref_out m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Trans-request>) ostream)
  "Serializes a message object of type '<Trans-request>"
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
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'delta))
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ref_out)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Trans-request>) istream)
  "Deserializes a message object of type '<Trans-request>"
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
  (cl:setf (cl:slot-value msg 'delta) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'delta)))
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
      (cl:setf (cl:slot-value msg 'ref_out) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Trans-request>)))
  "Returns string type for a service object of type '<Trans-request>"
  "dsr_msgs/TransRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trans-request)))
  "Returns string type for a service object of type 'Trans-request"
  "dsr_msgs/TransRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Trans-request>)))
  "Returns md5sum for a message object of type '<Trans-request>"
  "2cc8472f65a9eb8b986e1489e30025bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Trans-request)))
  "Returns md5sum for a message object of type 'Trans-request"
  "2cc8472f65a9eb8b986e1489e30025bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Trans-request>)))
  "Returns full string definition for message of type '<Trans-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# trans  ~%#____________________________________________________________________________________________~%~%float64[6] pos               # task pos(posx)  ~%float64[6] delta             # delta (posx)  ~%int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)~%                             # <DR_WORLD is only available in M2.40 or later> ~%int8       ref_out #= 0      # DR_BASE(0), DR_WORLD(2)~%                             # <ref_out is only available in M2.40 or later>~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Trans-request)))
  "Returns full string definition for message of type 'Trans-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# trans  ~%#____________________________________________________________________________________________~%~%float64[6] pos               # task pos(posx)  ~%float64[6] delta             # delta (posx)  ~%int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)~%                             # <DR_WORLD is only available in M2.40 or later> ~%int8       ref_out #= 0      # DR_BASE(0), DR_WORLD(2)~%                             # <ref_out is only available in M2.40 or later>~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Trans-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'delta) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Trans-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Trans-request
    (cl:cons ':pos (pos msg))
    (cl:cons ':delta (delta msg))
    (cl:cons ':ref (ref msg))
    (cl:cons ':ref_out (ref_out msg))
))
;//! \htmlinclude Trans-response.msg.html

(cl:defclass <Trans-response> (roslisp-msg-protocol:ros-message)
  ((trans_pos
    :reader trans_pos
    :initarg :trans_pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Trans-response (<Trans-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Trans-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Trans-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<Trans-response> is deprecated: use dsr_msgs-srv:Trans-response instead.")))

(cl:ensure-generic-function 'trans_pos-val :lambda-list '(m))
(cl:defmethod trans_pos-val ((m <Trans-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:trans_pos-val is deprecated.  Use dsr_msgs-srv:trans_pos instead.")
  (trans_pos m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Trans-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Trans-response>) ostream)
  "Serializes a message object of type '<Trans-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'trans_pos))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Trans-response>) istream)
  "Deserializes a message object of type '<Trans-response>"
  (cl:setf (cl:slot-value msg 'trans_pos) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'trans_pos)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Trans-response>)))
  "Returns string type for a service object of type '<Trans-response>"
  "dsr_msgs/TransResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trans-response)))
  "Returns string type for a service object of type 'Trans-response"
  "dsr_msgs/TransResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Trans-response>)))
  "Returns md5sum for a message object of type '<Trans-response>"
  "2cc8472f65a9eb8b986e1489e30025bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Trans-response)))
  "Returns md5sum for a message object of type 'Trans-response"
  "2cc8472f65a9eb8b986e1489e30025bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Trans-response>)))
  "Returns full string definition for message of type '<Trans-response>"
  (cl:format cl:nil "float64[6] trans_pos         # trans pos(posx) ~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Trans-response)))
  "Returns full string definition for message of type 'Trans-response"
  (cl:format cl:nil "float64[6] trans_pos         # trans pos(posx) ~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Trans-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'trans_pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Trans-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Trans-response
    (cl:cons ':trans_pos (trans_pos msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Trans)))
  'Trans-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Trans)))
  'Trans-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trans)))
  "Returns string type for a service object of type '<Trans>"
  "dsr_msgs/Trans")