; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude EnableAlterMotion-request.msg.html

(cl:defclass <EnableAlterMotion-request> (roslisp-msg-protocol:ros-message)
  ((n
    :reader n
    :initarg :n
    :type cl:integer
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0)
   (limit_dPOS
    :reader limit_dPOS
    :initarg :limit_dPOS
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (limit_dPOS_per
    :reader limit_dPOS_per
    :initarg :limit_dPOS_per
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass EnableAlterMotion-request (<EnableAlterMotion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EnableAlterMotion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EnableAlterMotion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<EnableAlterMotion-request> is deprecated: use dsr_msgs-srv:EnableAlterMotion-request instead.")))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <EnableAlterMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:n-val is deprecated.  Use dsr_msgs-srv:n instead.")
  (n m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <EnableAlterMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:mode-val is deprecated.  Use dsr_msgs-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <EnableAlterMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))

(cl:ensure-generic-function 'limit_dPOS-val :lambda-list '(m))
(cl:defmethod limit_dPOS-val ((m <EnableAlterMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:limit_dPOS-val is deprecated.  Use dsr_msgs-srv:limit_dPOS instead.")
  (limit_dPOS m))

(cl:ensure-generic-function 'limit_dPOS_per-val :lambda-list '(m))
(cl:defmethod limit_dPOS_per-val ((m <EnableAlterMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:limit_dPOS_per-val is deprecated.  Use dsr_msgs-srv:limit_dPOS_per instead.")
  (limit_dPOS_per m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EnableAlterMotion-request>) ostream)
  "Serializes a message object of type '<EnableAlterMotion-request>"
  (cl:let* ((signed (cl:slot-value msg 'n)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
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
   (cl:slot-value msg 'limit_dPOS))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'limit_dPOS_per))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EnableAlterMotion-request>) istream)
  "Deserializes a message object of type '<EnableAlterMotion-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:setf (cl:slot-value msg 'limit_dPOS) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'limit_dPOS)))
    (cl:dotimes (i 2)
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
  (cl:setf (cl:slot-value msg 'limit_dPOS_per) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'limit_dPOS_per)))
    (cl:dotimes (i 2)
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EnableAlterMotion-request>)))
  "Returns string type for a service object of type '<EnableAlterMotion-request>"
  "dsr_msgs/EnableAlterMotionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableAlterMotion-request)))
  "Returns string type for a service object of type 'EnableAlterMotion-request"
  "dsr_msgs/EnableAlterMotionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EnableAlterMotion-request>)))
  "Returns md5sum for a message object of type '<EnableAlterMotion-request>"
  "4bc090a8b26dd2353ca78fe889287b7f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EnableAlterMotion-request)))
  "Returns md5sum for a message object of type 'EnableAlterMotion-request"
  "4bc090a8b26dd2353ca78fe889287b7f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EnableAlterMotion-request>)))
  "Returns full string definition for message of type '<EnableAlterMotion-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# enable_alter_motion  ~%# ~%#____________________________________________________________________________________________~%~%int32      n                 # Cycle time number ~%int8       mode              # DR_DPOS(0) : accumulation amount, DR_DVEL(1) : increment amount ~%int8       ref               # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~~200) ~%                             # <ref is only available in M2.40 or later> ~%float64[2] limit_dPOS        # First value : limitation of position[mm], Second value : limitation of orientation[deg]~%float64[2] limit_dPOS_per    # First value : limitation of position[mm], Second value : limitation of orientation[deg]~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EnableAlterMotion-request)))
  "Returns full string definition for message of type 'EnableAlterMotion-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# enable_alter_motion  ~%# ~%#____________________________________________________________________________________________~%~%int32      n                 # Cycle time number ~%int8       mode              # DR_DPOS(0) : accumulation amount, DR_DVEL(1) : increment amount ~%int8       ref               # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~~200) ~%                             # <ref is only available in M2.40 or later> ~%float64[2] limit_dPOS        # First value : limitation of position[mm], Second value : limitation of orientation[deg]~%float64[2] limit_dPOS_per    # First value : limitation of position[mm], Second value : limitation of orientation[deg]~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EnableAlterMotion-request>))
  (cl:+ 0
     4
     1
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'limit_dPOS) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'limit_dPOS_per) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EnableAlterMotion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'EnableAlterMotion-request
    (cl:cons ':n (n msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':ref (ref msg))
    (cl:cons ':limit_dPOS (limit_dPOS msg))
    (cl:cons ':limit_dPOS_per (limit_dPOS_per msg))
))
;//! \htmlinclude EnableAlterMotion-response.msg.html

(cl:defclass <EnableAlterMotion-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass EnableAlterMotion-response (<EnableAlterMotion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EnableAlterMotion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EnableAlterMotion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<EnableAlterMotion-response> is deprecated: use dsr_msgs-srv:EnableAlterMotion-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <EnableAlterMotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EnableAlterMotion-response>) ostream)
  "Serializes a message object of type '<EnableAlterMotion-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EnableAlterMotion-response>) istream)
  "Deserializes a message object of type '<EnableAlterMotion-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EnableAlterMotion-response>)))
  "Returns string type for a service object of type '<EnableAlterMotion-response>"
  "dsr_msgs/EnableAlterMotionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableAlterMotion-response)))
  "Returns string type for a service object of type 'EnableAlterMotion-response"
  "dsr_msgs/EnableAlterMotionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EnableAlterMotion-response>)))
  "Returns md5sum for a message object of type '<EnableAlterMotion-response>"
  "4bc090a8b26dd2353ca78fe889287b7f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EnableAlterMotion-response)))
  "Returns md5sum for a message object of type 'EnableAlterMotion-response"
  "4bc090a8b26dd2353ca78fe889287b7f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EnableAlterMotion-response>)))
  "Returns full string definition for message of type '<EnableAlterMotion-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EnableAlterMotion-response)))
  "Returns full string definition for message of type 'EnableAlterMotion-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EnableAlterMotion-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EnableAlterMotion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'EnableAlterMotion-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'EnableAlterMotion)))
  'EnableAlterMotion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'EnableAlterMotion)))
  'EnableAlterMotion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableAlterMotion)))
  "Returns string type for a service object of type '<EnableAlterMotion>"
  "dsr_msgs/EnableAlterMotion")