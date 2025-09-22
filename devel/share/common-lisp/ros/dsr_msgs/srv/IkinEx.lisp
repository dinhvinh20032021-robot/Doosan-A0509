; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude IkinEx-request.msg.html

(cl:defclass <IkinEx-request> (roslisp-msg-protocol:ros-message)
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
    :initform 0)
   (ref_pos_opt
    :reader ref_pos_opt
    :initarg :ref_pos_opt
    :type cl:fixnum
    :initform 0))
)

(cl:defclass IkinEx-request (<IkinEx-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IkinEx-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IkinEx-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<IkinEx-request> is deprecated: use dsr_msgs-srv:IkinEx-request instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <IkinEx-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:pos-val is deprecated.  Use dsr_msgs-srv:pos instead.")
  (pos m))

(cl:ensure-generic-function 'sol_space-val :lambda-list '(m))
(cl:defmethod sol_space-val ((m <IkinEx-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:sol_space-val is deprecated.  Use dsr_msgs-srv:sol_space instead.")
  (sol_space m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <IkinEx-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))

(cl:ensure-generic-function 'ref_pos_opt-val :lambda-list '(m))
(cl:defmethod ref_pos_opt-val ((m <IkinEx-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref_pos_opt-val is deprecated.  Use dsr_msgs-srv:ref_pos_opt instead.")
  (ref_pos_opt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IkinEx-request>) ostream)
  "Serializes a message object of type '<IkinEx-request>"
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
  (cl:let* ((signed (cl:slot-value msg 'ref_pos_opt)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IkinEx-request>) istream)
  "Deserializes a message object of type '<IkinEx-request>"
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref_pos_opt) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IkinEx-request>)))
  "Returns string type for a service object of type '<IkinEx-request>"
  "dsr_msgs/IkinExRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IkinEx-request)))
  "Returns string type for a service object of type 'IkinEx-request"
  "dsr_msgs/IkinExRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IkinEx-request>)))
  "Returns md5sum for a message object of type '<IkinEx-request>"
  "52c2edae66404c90e27c09703f4690c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IkinEx-request)))
  "Returns md5sum for a message object of type 'IkinEx-request"
  "52c2edae66404c90e27c09703f4690c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IkinEx-request>)))
  "Returns full string definition for message of type '<IkinEx-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# ikin_ex  ~%#____________________________________________________________________________________________~%~%float64[6] pos               # task pos(posx)  ~%int8       sol_space         # solution space : 0 ~~ 7~%int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)~%int8       ref_pos_opt	     #  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IkinEx-request)))
  "Returns full string definition for message of type 'IkinEx-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# ikin_ex  ~%#____________________________________________________________________________________________~%~%float64[6] pos               # task pos(posx)  ~%int8       sol_space         # solution space : 0 ~~ 7~%int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)~%int8       ref_pos_opt	     #  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IkinEx-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IkinEx-request>))
  "Converts a ROS message object to a list"
  (cl:list 'IkinEx-request
    (cl:cons ':pos (pos msg))
    (cl:cons ':sol_space (sol_space msg))
    (cl:cons ':ref (ref msg))
    (cl:cons ':ref_pos_opt (ref_pos_opt msg))
))
;//! \htmlinclude IkinEx-response.msg.html

(cl:defclass <IkinEx-response> (roslisp-msg-protocol:ros-message)
  ((conv_posj
    :reader conv_posj
    :initarg :conv_posj
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass IkinEx-response (<IkinEx-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IkinEx-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IkinEx-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<IkinEx-response> is deprecated: use dsr_msgs-srv:IkinEx-response instead.")))

(cl:ensure-generic-function 'conv_posj-val :lambda-list '(m))
(cl:defmethod conv_posj-val ((m <IkinEx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:conv_posj-val is deprecated.  Use dsr_msgs-srv:conv_posj instead.")
  (conv_posj m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <IkinEx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:status-val is deprecated.  Use dsr_msgs-srv:status instead.")
  (status m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <IkinEx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IkinEx-response>) ostream)
  "Serializes a message object of type '<IkinEx-response>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IkinEx-response>) istream)
  "Deserializes a message object of type '<IkinEx-response>"
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
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IkinEx-response>)))
  "Returns string type for a service object of type '<IkinEx-response>"
  "dsr_msgs/IkinExResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IkinEx-response)))
  "Returns string type for a service object of type 'IkinEx-response"
  "dsr_msgs/IkinExResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IkinEx-response>)))
  "Returns md5sum for a message object of type '<IkinEx-response>"
  "52c2edae66404c90e27c09703f4690c1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IkinEx-response)))
  "Returns md5sum for a message object of type 'IkinEx-response"
  "52c2edae66404c90e27c09703f4690c1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IkinEx-response>)))
  "Returns full string definition for message of type '<IkinEx-response>"
  (cl:format cl:nil "float64[6] conv_posj         # joint pos(posj)  ~%bool       status~%bool       success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IkinEx-response)))
  "Returns full string definition for message of type 'IkinEx-response"
  (cl:format cl:nil "float64[6] conv_posj         # joint pos(posj)  ~%bool       status~%bool       success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IkinEx-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'conv_posj) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IkinEx-response>))
  "Converts a ROS message object to a list"
  (cl:list 'IkinEx-response
    (cl:cons ':conv_posj (conv_posj msg))
    (cl:cons ':status (status msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'IkinEx)))
  'IkinEx-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'IkinEx)))
  'IkinEx-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IkinEx)))
  "Returns string type for a service object of type '<IkinEx>"
  "dsr_msgs/IkinEx")