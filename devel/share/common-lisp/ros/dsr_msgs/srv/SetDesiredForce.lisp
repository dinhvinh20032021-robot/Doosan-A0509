; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetDesiredForce-request.msg.html

(cl:defclass <SetDesiredForce-request> (roslisp-msg-protocol:ros-message)
  ((fd
    :reader fd
    :initarg :fd
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (dir
    :reader dir
    :initarg :dir
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 6 :element-type 'cl:fixnum :initial-element 0))
   (ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0)
   (time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0)
   (mod
    :reader mod
    :initarg :mod
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetDesiredForce-request (<SetDesiredForce-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDesiredForce-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDesiredForce-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetDesiredForce-request> is deprecated: use dsr_msgs-srv:SetDesiredForce-request instead.")))

(cl:ensure-generic-function 'fd-val :lambda-list '(m))
(cl:defmethod fd-val ((m <SetDesiredForce-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:fd-val is deprecated.  Use dsr_msgs-srv:fd instead.")
  (fd m))

(cl:ensure-generic-function 'dir-val :lambda-list '(m))
(cl:defmethod dir-val ((m <SetDesiredForce-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:dir-val is deprecated.  Use dsr_msgs-srv:dir instead.")
  (dir m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <SetDesiredForce-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <SetDesiredForce-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:time-val is deprecated.  Use dsr_msgs-srv:time instead.")
  (time m))

(cl:ensure-generic-function 'mod-val :lambda-list '(m))
(cl:defmethod mod-val ((m <SetDesiredForce-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:mod-val is deprecated.  Use dsr_msgs-srv:mod instead.")
  (mod m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDesiredForce-request>) ostream)
  "Serializes a message object of type '<SetDesiredForce-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'fd))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'dir))
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'mod)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDesiredForce-request>) istream)
  "Deserializes a message object of type '<SetDesiredForce-request>"
  (cl:setf (cl:slot-value msg 'fd) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'fd)))
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
  (cl:setf (cl:slot-value msg 'dir) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'dir)))
    (cl:dotimes (i 6)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mod) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDesiredForce-request>)))
  "Returns string type for a service object of type '<SetDesiredForce-request>"
  "dsr_msgs/SetDesiredForceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDesiredForce-request)))
  "Returns string type for a service object of type 'SetDesiredForce-request"
  "dsr_msgs/SetDesiredForceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDesiredForce-request>)))
  "Returns md5sum for a message object of type '<SetDesiredForce-request>"
  "06a47c6fc3ae37ec04174ee64d651e04")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDesiredForce-request)))
  "Returns md5sum for a message object of type 'SetDesiredForce-request"
  "06a47c6fc3ae37ec04174ee64d651e04")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDesiredForce-request>)))
  "Returns full string definition for message of type '<SetDesiredForce-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_desired_force  ~%#____________________________________________________________________________________________~%~%float64[6] fd                # Three translational target forces + Three rotational target moments~%int8[6]    dir               # Force control in the corresponding direction if 1, Compliance control in the corresponding direction if 0~%int8       ref               # Reference coordinate of the coordinate to get~%float64    time # 0          # Transition time of target force to take effect (0 ~~ 1.0 sec)~%int8       mod               # DR_FC_MOD_ABS(0): force control with absolute value, ~%                             # DR_FC_MOD_REL(1): force control with relative value to initial state (the instance when this function is called) ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDesiredForce-request)))
  "Returns full string definition for message of type 'SetDesiredForce-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_desired_force  ~%#____________________________________________________________________________________________~%~%float64[6] fd                # Three translational target forces + Three rotational target moments~%int8[6]    dir               # Force control in the corresponding direction if 1, Compliance control in the corresponding direction if 0~%int8       ref               # Reference coordinate of the coordinate to get~%float64    time # 0          # Transition time of target force to take effect (0 ~~ 1.0 sec)~%int8       mod               # DR_FC_MOD_ABS(0): force control with absolute value, ~%                             # DR_FC_MOD_REL(1): force control with relative value to initial state (the instance when this function is called) ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDesiredForce-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'fd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'dir) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDesiredForce-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDesiredForce-request
    (cl:cons ':fd (fd msg))
    (cl:cons ':dir (dir msg))
    (cl:cons ':ref (ref msg))
    (cl:cons ':time (time msg))
    (cl:cons ':mod (mod msg))
))
;//! \htmlinclude SetDesiredForce-response.msg.html

(cl:defclass <SetDesiredForce-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetDesiredForce-response (<SetDesiredForce-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDesiredForce-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDesiredForce-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetDesiredForce-response> is deprecated: use dsr_msgs-srv:SetDesiredForce-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetDesiredForce-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDesiredForce-response>) ostream)
  "Serializes a message object of type '<SetDesiredForce-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDesiredForce-response>) istream)
  "Deserializes a message object of type '<SetDesiredForce-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDesiredForce-response>)))
  "Returns string type for a service object of type '<SetDesiredForce-response>"
  "dsr_msgs/SetDesiredForceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDesiredForce-response)))
  "Returns string type for a service object of type 'SetDesiredForce-response"
  "dsr_msgs/SetDesiredForceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDesiredForce-response>)))
  "Returns md5sum for a message object of type '<SetDesiredForce-response>"
  "06a47c6fc3ae37ec04174ee64d651e04")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDesiredForce-response)))
  "Returns md5sum for a message object of type 'SetDesiredForce-response"
  "06a47c6fc3ae37ec04174ee64d651e04")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDesiredForce-response>)))
  "Returns full string definition for message of type '<SetDesiredForce-response>"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDesiredForce-response)))
  "Returns full string definition for message of type 'SetDesiredForce-response"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDesiredForce-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDesiredForce-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDesiredForce-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetDesiredForce)))
  'SetDesiredForce-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetDesiredForce)))
  'SetDesiredForce-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDesiredForce)))
  "Returns string type for a service object of type '<SetDesiredForce>"
  "dsr_msgs/SetDesiredForce")