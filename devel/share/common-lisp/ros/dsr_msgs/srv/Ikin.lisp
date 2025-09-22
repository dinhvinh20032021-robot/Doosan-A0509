; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude Ikin-request.msg.html

(cl:defclass <Ikin-request> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (sol_space
    :reader sol_space
    :initarg :sol_space
    :type cl:fixnum
    :initform 0)
   (ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Ikin-request (<Ikin-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Ikin-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Ikin-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<Ikin-request> is deprecated: use dsr_msgs-srv:Ikin-request instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <Ikin-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:pos-val is deprecated.  Use dsr_msgs-srv:pos instead.")
  (pos m))

(cl:ensure-generic-function 'sol_space-val :lambda-list '(m))
(cl:defmethod sol_space-val ((m <Ikin-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:sol_space-val is deprecated.  Use dsr_msgs-srv:sol_space instead.")
  (sol_space m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <Ikin-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Ikin-request>) ostream)
  "Serializes a message object of type '<Ikin-request>"
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
  (cl:let* ((signed (cl:slot-value msg 'sol_space)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Ikin-request>) istream)
  "Deserializes a message object of type '<Ikin-request>"
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
      (cl:setf (cl:slot-value msg 'sol_space) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Ikin-request>)))
  "Returns string type for a service object of type '<Ikin-request>"
  "dsr_msgs/IkinRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ikin-request)))
  "Returns string type for a service object of type 'Ikin-request"
  "dsr_msgs/IkinRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Ikin-request>)))
  "Returns md5sum for a message object of type '<Ikin-request>"
  "fb475e378a19f7cef6d8ac54d5b6fb72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Ikin-request)))
  "Returns md5sum for a message object of type 'Ikin-request"
  "fb475e378a19f7cef6d8ac54d5b6fb72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Ikin-request>)))
  "Returns full string definition for message of type '<Ikin-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# ikin  ~%#____________________________________________________________________________________________~%~%float64[6] pos               # task pos(posx)  ~%int8       sol_space         # solution space : 0 ~~ 7~%int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Ikin-request)))
  "Returns full string definition for message of type 'Ikin-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# ikin  ~%#____________________________________________________________________________________________~%~%float64[6] pos               # task pos(posx)  ~%int8       sol_space         # solution space : 0 ~~ 7~%int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Ikin-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Ikin-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Ikin-request
    (cl:cons ':pos (pos msg))
    (cl:cons ':sol_space (sol_space msg))
    (cl:cons ':ref (ref msg))
))
;//! \htmlinclude Ikin-response.msg.html

(cl:defclass <Ikin-response> (roslisp-msg-protocol:ros-message)
  ((conv_posj
    :reader conv_posj
    :initarg :conv_posj
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Ikin-response (<Ikin-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Ikin-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Ikin-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<Ikin-response> is deprecated: use dsr_msgs-srv:Ikin-response instead.")))

(cl:ensure-generic-function 'conv_posj-val :lambda-list '(m))
(cl:defmethod conv_posj-val ((m <Ikin-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:conv_posj-val is deprecated.  Use dsr_msgs-srv:conv_posj instead.")
  (conv_posj m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Ikin-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Ikin-response>) ostream)
  "Serializes a message object of type '<Ikin-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'conv_posj))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Ikin-response>) istream)
  "Deserializes a message object of type '<Ikin-response>"
  (cl:setf (cl:slot-value msg 'conv_posj) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'conv_posj)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Ikin-response>)))
  "Returns string type for a service object of type '<Ikin-response>"
  "dsr_msgs/IkinResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ikin-response)))
  "Returns string type for a service object of type 'Ikin-response"
  "dsr_msgs/IkinResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Ikin-response>)))
  "Returns md5sum for a message object of type '<Ikin-response>"
  "fb475e378a19f7cef6d8ac54d5b6fb72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Ikin-response)))
  "Returns md5sum for a message object of type 'Ikin-response"
  "fb475e378a19f7cef6d8ac54d5b6fb72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Ikin-response>)))
  "Returns full string definition for message of type '<Ikin-response>"
  (cl:format cl:nil "float64[6] conv_posj         # joint pos(posj)  ~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Ikin-response)))
  "Returns full string definition for message of type 'Ikin-response"
  (cl:format cl:nil "float64[6] conv_posj         # joint pos(posj)  ~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Ikin-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'conv_posj) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Ikin-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Ikin-response
    (cl:cons ':conv_posj (conv_posj msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Ikin)))
  'Ikin-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Ikin)))
  'Ikin-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ikin)))
  "Returns string type for a service object of type '<Ikin>"
  "dsr_msgs/Ikin")