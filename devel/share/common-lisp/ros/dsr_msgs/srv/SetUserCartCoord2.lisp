; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetUserCartCoord2-request.msg.html

(cl:defclass <SetUserCartCoord2-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetUserCartCoord2-request (<SetUserCartCoord2-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetUserCartCoord2-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetUserCartCoord2-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetUserCartCoord2-request> is deprecated: use dsr_msgs-srv:SetUserCartCoord2-request instead.")))

(cl:ensure-generic-function 'x1-val :lambda-list '(m))
(cl:defmethod x1-val ((m <SetUserCartCoord2-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:x1-val is deprecated.  Use dsr_msgs-srv:x1 instead.")
  (x1 m))

(cl:ensure-generic-function 'x2-val :lambda-list '(m))
(cl:defmethod x2-val ((m <SetUserCartCoord2-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:x2-val is deprecated.  Use dsr_msgs-srv:x2 instead.")
  (x2 m))

(cl:ensure-generic-function 'x3-val :lambda-list '(m))
(cl:defmethod x3-val ((m <SetUserCartCoord2-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:x3-val is deprecated.  Use dsr_msgs-srv:x3 instead.")
  (x3 m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <SetUserCartCoord2-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:pos-val is deprecated.  Use dsr_msgs-srv:pos instead.")
  (pos m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <SetUserCartCoord2-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetUserCartCoord2-request>) ostream)
  "Serializes a message object of type '<SetUserCartCoord2-request>"
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
   (cl:slot-value msg 'pos))
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetUserCartCoord2-request>) istream)
  "Deserializes a message object of type '<SetUserCartCoord2-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetUserCartCoord2-request>)))
  "Returns string type for a service object of type '<SetUserCartCoord2-request>"
  "dsr_msgs/SetUserCartCoord2Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUserCartCoord2-request)))
  "Returns string type for a service object of type 'SetUserCartCoord2-request"
  "dsr_msgs/SetUserCartCoord2Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetUserCartCoord2-request>)))
  "Returns md5sum for a message object of type '<SetUserCartCoord2-request>"
  "42d11c6c94c1dfc41a37882dbd6da758")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetUserCartCoord2-request)))
  "Returns md5sum for a message object of type 'SetUserCartCoord2-request"
  "42d11c6c94c1dfc41a37882dbd6da758")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetUserCartCoord2-request>)))
  "Returns full string definition for message of type '<SetUserCartCoord2-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_user_cart_coord(x1, x2, x3, pos, ref)~%#____________________________________________________________________________________________~%~%float64[6] x1                 # task pos(posx)  ~%float64[6] x2                 # task pos(posx)  ~%float64[6] x3                 # task pos(posx)~%float64[6] pos                # pos(posx)~%int8       ref                # DR_BASE(0), DR_WORLD(2)~%                              # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetUserCartCoord2-request)))
  "Returns full string definition for message of type 'SetUserCartCoord2-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_user_cart_coord(x1, x2, x3, pos, ref)~%#____________________________________________________________________________________________~%~%float64[6] x1                 # task pos(posx)  ~%float64[6] x2                 # task pos(posx)  ~%float64[6] x3                 # task pos(posx)~%float64[6] pos                # pos(posx)~%int8       ref                # DR_BASE(0), DR_WORLD(2)~%                              # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetUserCartCoord2-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'x1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'x2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'x3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetUserCartCoord2-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetUserCartCoord2-request
    (cl:cons ':x1 (x1 msg))
    (cl:cons ':x2 (x2 msg))
    (cl:cons ':x3 (x3 msg))
    (cl:cons ':pos (pos msg))
    (cl:cons ':ref (ref msg))
))
;//! \htmlinclude SetUserCartCoord2-response.msg.html

(cl:defclass <SetUserCartCoord2-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetUserCartCoord2-response (<SetUserCartCoord2-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetUserCartCoord2-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetUserCartCoord2-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetUserCartCoord2-response> is deprecated: use dsr_msgs-srv:SetUserCartCoord2-response instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SetUserCartCoord2-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:id-val is deprecated.  Use dsr_msgs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetUserCartCoord2-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetUserCartCoord2-response>) ostream)
  "Serializes a message object of type '<SetUserCartCoord2-response>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetUserCartCoord2-response>) istream)
  "Deserializes a message object of type '<SetUserCartCoord2-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetUserCartCoord2-response>)))
  "Returns string type for a service object of type '<SetUserCartCoord2-response>"
  "dsr_msgs/SetUserCartCoord2Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUserCartCoord2-response)))
  "Returns string type for a service object of type 'SetUserCartCoord2-response"
  "dsr_msgs/SetUserCartCoord2Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetUserCartCoord2-response>)))
  "Returns md5sum for a message object of type '<SetUserCartCoord2-response>"
  "42d11c6c94c1dfc41a37882dbd6da758")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetUserCartCoord2-response)))
  "Returns md5sum for a message object of type 'SetUserCartCoord2-response"
  "42d11c6c94c1dfc41a37882dbd6da758")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetUserCartCoord2-response>)))
  "Returns full string definition for message of type '<SetUserCartCoord2-response>"
  (cl:format cl:nil "int8    id                    # set user coord (101~~200) or fail(-1) ~%bool        success   ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetUserCartCoord2-response)))
  "Returns full string definition for message of type 'SetUserCartCoord2-response"
  (cl:format cl:nil "int8    id                    # set user coord (101~~200) or fail(-1) ~%bool        success   ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetUserCartCoord2-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetUserCartCoord2-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetUserCartCoord2-response
    (cl:cons ':id (id msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetUserCartCoord2)))
  'SetUserCartCoord2-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetUserCartCoord2)))
  'SetUserCartCoord2-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetUserCartCoord2)))
  "Returns string type for a service object of type '<SetUserCartCoord2>"
  "dsr_msgs/SetUserCartCoord2")