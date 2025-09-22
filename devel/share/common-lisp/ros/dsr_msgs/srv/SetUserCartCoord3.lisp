; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetUserCartCoord3-request.msg.html

(cl:defclass <SetUserCartCoord3-request> (roslisp-msg-protocol:ros-message)
  ((u1
    :reader u1
    :initarg :u1
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (v1
    :reader v1
    :initarg :v1
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (pos
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

(cl:defclass SetUserCartCoord3-request (<SetUserCartCoord3-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetUserCartCoord3-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetUserCartCoord3-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetUserCartCoord3-request> is deprecated: use dsr_msgs-srv:SetUserCartCoord3-request instead.")))

(cl:ensure-generic-function 'u1-val :lambda-list '(m))
(cl:defmethod u1-val ((m <SetUserCartCoord3-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:u1-val is deprecated.  Use dsr_msgs-srv:u1 instead.")
  (u1 m))

(cl:ensure-generic-function 'v1-val :lambda-list '(m))
(cl:defmethod v1-val ((m <SetUserCartCoord3-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:v1-val is deprecated.  Use dsr_msgs-srv:v1 instead.")
  (v1 m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <SetUserCartCoord3-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:pos-val is deprecated.  Use dsr_msgs-srv:pos instead.")
  (pos m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <SetUserCartCoord3-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetUserCartCoord3-request>) ostream)
  "Serializes a message object of type '<SetUserCartCoord3-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'u1))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'v1))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetUserCartCoord3-request>) istream)
  "Deserializes a message object of type '<SetUserCartCoord3-request>"
  (cl:setf (cl:slot-value msg 'u1) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'u1)))
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
  (cl:setf (cl:slot-value msg 'v1) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'v1)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetUserCartCoord3-request>)))
  "Returns string type for a service object of type '<SetUserCartCoord3-request>"
  "dsr_msgs/SetUserCartCoord3Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUserCartCoord3-request)))
  "Returns string type for a service object of type 'SetUserCartCoord3-request"
  "dsr_msgs/SetUserCartCoord3Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetUserCartCoord3-request>)))
  "Returns md5sum for a message object of type '<SetUserCartCoord3-request>"
  "c6fddb22deca3c301bea04b53df0d0a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetUserCartCoord3-request)))
  "Returns md5sum for a message object of type 'SetUserCartCoord3-request"
  "c6fddb22deca3c301bea04b53df0d0a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetUserCartCoord3-request>)))
  "Returns full string definition for message of type '<SetUserCartCoord3-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_user_cart_coord(u1, v1, pos, ref) ~%#____________________________________________________________________________________________~%~%float64[3] u1                # X-axis unit vector  ~%float64[3] v1                # Y-axis unit vector ~%float64[6] pos               # task pos(posx) ~%int8       ref               # DR_BASE(0), DR_WORLD(2)~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetUserCartCoord3-request)))
  "Returns full string definition for message of type 'SetUserCartCoord3-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_user_cart_coord(u1, v1, pos, ref) ~%#____________________________________________________________________________________________~%~%float64[3] u1                # X-axis unit vector  ~%float64[3] v1                # Y-axis unit vector ~%float64[6] pos               # task pos(posx) ~%int8       ref               # DR_BASE(0), DR_WORLD(2)~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetUserCartCoord3-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'u1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'v1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetUserCartCoord3-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetUserCartCoord3-request
    (cl:cons ':u1 (u1 msg))
    (cl:cons ':v1 (v1 msg))
    (cl:cons ':pos (pos msg))
    (cl:cons ':ref (ref msg))
))
;//! \htmlinclude SetUserCartCoord3-response.msg.html

(cl:defclass <SetUserCartCoord3-response> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetUserCartCoord3-response (<SetUserCartCoord3-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetUserCartCoord3-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetUserCartCoord3-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetUserCartCoord3-response> is deprecated: use dsr_msgs-srv:SetUserCartCoord3-response instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SetUserCartCoord3-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:id-val is deprecated.  Use dsr_msgs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetUserCartCoord3-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetUserCartCoord3-response>) ostream)
  "Serializes a message object of type '<SetUserCartCoord3-response>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetUserCartCoord3-response>) istream)
  "Deserializes a message object of type '<SetUserCartCoord3-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetUserCartCoord3-response>)))
  "Returns string type for a service object of type '<SetUserCartCoord3-response>"
  "dsr_msgs/SetUserCartCoord3Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUserCartCoord3-response)))
  "Returns string type for a service object of type 'SetUserCartCoord3-response"
  "dsr_msgs/SetUserCartCoord3Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetUserCartCoord3-response>)))
  "Returns md5sum for a message object of type '<SetUserCartCoord3-response>"
  "c6fddb22deca3c301bea04b53df0d0a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetUserCartCoord3-response)))
  "Returns md5sum for a message object of type 'SetUserCartCoord3-response"
  "c6fddb22deca3c301bea04b53df0d0a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetUserCartCoord3-response>)))
  "Returns full string definition for message of type '<SetUserCartCoord3-response>"
  (cl:format cl:nil "int8    id                   # set user coord (101~~120) or fail(-1) ~%bool        success   ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetUserCartCoord3-response)))
  "Returns full string definition for message of type 'SetUserCartCoord3-response"
  (cl:format cl:nil "int8    id                   # set user coord (101~~120) or fail(-1) ~%bool        success   ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetUserCartCoord3-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetUserCartCoord3-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetUserCartCoord3-response
    (cl:cons ':id (id msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetUserCartCoord3)))
  'SetUserCartCoord3-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetUserCartCoord3)))
  'SetUserCartCoord3-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUserCartCoord3)))
  "Returns string type for a service object of type '<SetUserCartCoord3>"
  "dsr_msgs/SetUserCartCoord3")