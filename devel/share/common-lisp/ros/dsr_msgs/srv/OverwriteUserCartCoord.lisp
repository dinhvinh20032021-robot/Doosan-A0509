; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude OverwriteUserCartCoord-request.msg.html

(cl:defclass <OverwriteUserCartCoord-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
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

(cl:defclass OverwriteUserCartCoord-request (<OverwriteUserCartCoord-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OverwriteUserCartCoord-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OverwriteUserCartCoord-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<OverwriteUserCartCoord-request> is deprecated: use dsr_msgs-srv:OverwriteUserCartCoord-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <OverwriteUserCartCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:id-val is deprecated.  Use dsr_msgs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <OverwriteUserCartCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:pos-val is deprecated.  Use dsr_msgs-srv:pos instead.")
  (pos m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <OverwriteUserCartCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OverwriteUserCartCoord-request>) ostream)
  "Serializes a message object of type '<OverwriteUserCartCoord-request>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
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
   (cl:slot-value msg 'pos))
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OverwriteUserCartCoord-request>) istream)
  "Deserializes a message object of type '<OverwriteUserCartCoord-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OverwriteUserCartCoord-request>)))
  "Returns string type for a service object of type '<OverwriteUserCartCoord-request>"
  "dsr_msgs/OverwriteUserCartCoordRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OverwriteUserCartCoord-request)))
  "Returns string type for a service object of type 'OverwriteUserCartCoord-request"
  "dsr_msgs/OverwriteUserCartCoordRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OverwriteUserCartCoord-request>)))
  "Returns md5sum for a message object of type '<OverwriteUserCartCoord-request>"
  "dd70ea2a3d7fe4aee2fb401824577ffd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OverwriteUserCartCoord-request)))
  "Returns md5sum for a message object of type 'OverwriteUserCartCoord-request"
  "dd70ea2a3d7fe4aee2fb401824577ffd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OverwriteUserCartCoord-request>)))
  "Returns full string definition for message of type '<OverwriteUserCartCoord-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# overwrite_user_cart_coord   ~%#____________________________________________________________________________________________~%# This service is only available in M2.50 or later~%~%int8       id                # ID of user coord ~%float64[6] pos               # task pos(posx)  ~%int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OverwriteUserCartCoord-request)))
  "Returns full string definition for message of type 'OverwriteUserCartCoord-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# overwrite_user_cart_coord   ~%#____________________________________________________________________________________________~%# This service is only available in M2.50 or later~%~%int8       id                # ID of user coord ~%float64[6] pos               # task pos(posx)  ~%int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OverwriteUserCartCoord-request>))
  (cl:+ 0
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OverwriteUserCartCoord-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OverwriteUserCartCoord-request
    (cl:cons ':id (id msg))
    (cl:cons ':pos (pos msg))
    (cl:cons ':ref (ref msg))
))
;//! \htmlinclude OverwriteUserCartCoord-response.msg.html

(cl:defclass <OverwriteUserCartCoord-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass OverwriteUserCartCoord-response (<OverwriteUserCartCoord-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OverwriteUserCartCoord-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OverwriteUserCartCoord-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<OverwriteUserCartCoord-response> is deprecated: use dsr_msgs-srv:OverwriteUserCartCoord-response instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <OverwriteUserCartCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:id-val is deprecated.  Use dsr_msgs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <OverwriteUserCartCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OverwriteUserCartCoord-response>) ostream)
  "Serializes a message object of type '<OverwriteUserCartCoord-response>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OverwriteUserCartCoord-response>) istream)
  "Deserializes a message object of type '<OverwriteUserCartCoord-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OverwriteUserCartCoord-response>)))
  "Returns string type for a service object of type '<OverwriteUserCartCoord-response>"
  "dsr_msgs/OverwriteUserCartCoordResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OverwriteUserCartCoord-response)))
  "Returns string type for a service object of type 'OverwriteUserCartCoord-response"
  "dsr_msgs/OverwriteUserCartCoordResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OverwriteUserCartCoord-response>)))
  "Returns md5sum for a message object of type '<OverwriteUserCartCoord-response>"
  "dd70ea2a3d7fe4aee2fb401824577ffd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OverwriteUserCartCoord-response)))
  "Returns md5sum for a message object of type 'OverwriteUserCartCoord-response"
  "dd70ea2a3d7fe4aee2fb401824577ffd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OverwriteUserCartCoord-response>)))
  "Returns full string definition for message of type '<OverwriteUserCartCoord-response>"
  (cl:format cl:nil "int8       id                # Successful coordinate setting, Set user coordinate ID (101 - 200)~%                             # (-1) Failed coordinate setting~%bool       success                             ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OverwriteUserCartCoord-response)))
  "Returns full string definition for message of type 'OverwriteUserCartCoord-response"
  (cl:format cl:nil "int8       id                # Successful coordinate setting, Set user coordinate ID (101 - 200)~%                             # (-1) Failed coordinate setting~%bool       success                             ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OverwriteUserCartCoord-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OverwriteUserCartCoord-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OverwriteUserCartCoord-response
    (cl:cons ':id (id msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OverwriteUserCartCoord)))
  'OverwriteUserCartCoord-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OverwriteUserCartCoord)))
  'OverwriteUserCartCoord-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OverwriteUserCartCoord)))
  "Returns string type for a service object of type '<OverwriteUserCartCoord>"
  "dsr_msgs/OverwriteUserCartCoord")