; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetStiffnessx-request.msg.html

(cl:defclass <SetStiffnessx-request> (roslisp-msg-protocol:ros-message)
  ((stx
    :reader stx
    :initarg :stx
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0)
   (time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetStiffnessx-request (<SetStiffnessx-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStiffnessx-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStiffnessx-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetStiffnessx-request> is deprecated: use dsr_msgs-srv:SetStiffnessx-request instead.")))

(cl:ensure-generic-function 'stx-val :lambda-list '(m))
(cl:defmethod stx-val ((m <SetStiffnessx-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:stx-val is deprecated.  Use dsr_msgs-srv:stx instead.")
  (stx m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <SetStiffnessx-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <SetStiffnessx-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:time-val is deprecated.  Use dsr_msgs-srv:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStiffnessx-request>) ostream)
  "Serializes a message object of type '<SetStiffnessx-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'stx))
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStiffnessx-request>) istream)
  "Deserializes a message object of type '<SetStiffnessx-request>"
  (cl:setf (cl:slot-value msg 'stx) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'stx)))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStiffnessx-request>)))
  "Returns string type for a service object of type '<SetStiffnessx-request>"
  "dsr_msgs/SetStiffnessxRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStiffnessx-request)))
  "Returns string type for a service object of type 'SetStiffnessx-request"
  "dsr_msgs/SetStiffnessxRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStiffnessx-request>)))
  "Returns md5sum for a message object of type '<SetStiffnessx-request>"
  "13f8b8d245232e98ce000bd4e6262bcd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStiffnessx-request)))
  "Returns md5sum for a message object of type 'SetStiffnessx-request"
  "13f8b8d245232e98ce000bd4e6262bcd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStiffnessx-request>)))
  "Returns full string definition for message of type '<SetStiffnessx-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_stiffnessx  ~%#____________________________________________________________________________________________~%~%float64[6] stx               # default[500, 500, 500, 100, 100, 100], Three translational stiffnesses + Three rotational stiffnesses~%int8       ref               # the preset reference coordinate system.~%float64    time              # Stiffness varying time(0 ~~ 1.0) [sec], Linear transition during the specified time   ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStiffnessx-request)))
  "Returns full string definition for message of type 'SetStiffnessx-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_stiffnessx  ~%#____________________________________________________________________________________________~%~%float64[6] stx               # default[500, 500, 500, 100, 100, 100], Three translational stiffnesses + Three rotational stiffnesses~%int8       ref               # the preset reference coordinate system.~%float64    time              # Stiffness varying time(0 ~~ 1.0) [sec], Linear transition during the specified time   ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStiffnessx-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'stx) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStiffnessx-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStiffnessx-request
    (cl:cons ':stx (stx msg))
    (cl:cons ':ref (ref msg))
    (cl:cons ':time (time msg))
))
;//! \htmlinclude SetStiffnessx-response.msg.html

(cl:defclass <SetStiffnessx-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetStiffnessx-response (<SetStiffnessx-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStiffnessx-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStiffnessx-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetStiffnessx-response> is deprecated: use dsr_msgs-srv:SetStiffnessx-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetStiffnessx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStiffnessx-response>) ostream)
  "Serializes a message object of type '<SetStiffnessx-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStiffnessx-response>) istream)
  "Deserializes a message object of type '<SetStiffnessx-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStiffnessx-response>)))
  "Returns string type for a service object of type '<SetStiffnessx-response>"
  "dsr_msgs/SetStiffnessxResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStiffnessx-response)))
  "Returns string type for a service object of type 'SetStiffnessx-response"
  "dsr_msgs/SetStiffnessxResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStiffnessx-response>)))
  "Returns md5sum for a message object of type '<SetStiffnessx-response>"
  "13f8b8d245232e98ce000bd4e6262bcd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStiffnessx-response)))
  "Returns md5sum for a message object of type 'SetStiffnessx-response"
  "13f8b8d245232e98ce000bd4e6262bcd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStiffnessx-response>)))
  "Returns full string definition for message of type '<SetStiffnessx-response>"
  (cl:format cl:nil "bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStiffnessx-response)))
  "Returns full string definition for message of type 'SetStiffnessx-response"
  (cl:format cl:nil "bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStiffnessx-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStiffnessx-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStiffnessx-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetStiffnessx)))
  'SetStiffnessx-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetStiffnessx)))
  'SetStiffnessx-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStiffnessx)))
  "Returns string type for a service object of type '<SetStiffnessx>"
  "dsr_msgs/SetStiffnessx")