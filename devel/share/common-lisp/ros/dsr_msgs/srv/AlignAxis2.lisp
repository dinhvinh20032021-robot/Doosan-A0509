; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude AlignAxis2-request.msg.html

(cl:defclass <AlignAxis2-request> (roslisp-msg-protocol:ros-message)
  ((target_vect
    :reader target_vect
    :initarg :target_vect
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (source_vect
    :reader source_vect
    :initarg :source_vect
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (axis
    :reader axis
    :initarg :axis
    :type cl:fixnum
    :initform 0)
   (ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AlignAxis2-request (<AlignAxis2-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AlignAxis2-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AlignAxis2-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<AlignAxis2-request> is deprecated: use dsr_msgs-srv:AlignAxis2-request instead.")))

(cl:ensure-generic-function 'target_vect-val :lambda-list '(m))
(cl:defmethod target_vect-val ((m <AlignAxis2-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:target_vect-val is deprecated.  Use dsr_msgs-srv:target_vect instead.")
  (target_vect m))

(cl:ensure-generic-function 'source_vect-val :lambda-list '(m))
(cl:defmethod source_vect-val ((m <AlignAxis2-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:source_vect-val is deprecated.  Use dsr_msgs-srv:source_vect instead.")
  (source_vect m))

(cl:ensure-generic-function 'axis-val :lambda-list '(m))
(cl:defmethod axis-val ((m <AlignAxis2-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:axis-val is deprecated.  Use dsr_msgs-srv:axis instead.")
  (axis m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <AlignAxis2-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AlignAxis2-request>) ostream)
  "Serializes a message object of type '<AlignAxis2-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'target_vect))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'source_vect))
  (cl:let* ((signed (cl:slot-value msg 'axis)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AlignAxis2-request>) istream)
  "Deserializes a message object of type '<AlignAxis2-request>"
  (cl:setf (cl:slot-value msg 'target_vect) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'target_vect)))
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
  (cl:setf (cl:slot-value msg 'source_vect) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'source_vect)))
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'axis) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AlignAxis2-request>)))
  "Returns string type for a service object of type '<AlignAxis2-request>"
  "dsr_msgs/AlignAxis2Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AlignAxis2-request)))
  "Returns string type for a service object of type 'AlignAxis2-request"
  "dsr_msgs/AlignAxis2Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AlignAxis2-request>)))
  "Returns md5sum for a message object of type '<AlignAxis2-request>"
  "789e8de44f005548a96c6375678d766c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AlignAxis2-request)))
  "Returns md5sum for a message object of type 'AlignAxis2-request"
  "789e8de44f005548a96c6375678d766c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AlignAxis2-request>)))
  "Returns full string definition for message of type '<AlignAxis2-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# align_axis(target_vect, source_vect, axis, ref)~%#____________________________________________________________________________________________~%~%float64[3] target_vect       # target vector[3]  ~%float64[3] source_vect       # source vector[3]  ~%int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) ~%int8       ref               # DR_BASE(0), DR_WORLD(2), user coord(101~~200)~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AlignAxis2-request)))
  "Returns full string definition for message of type 'AlignAxis2-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# align_axis(target_vect, source_vect, axis, ref)~%#____________________________________________________________________________________________~%~%float64[3] target_vect       # target vector[3]  ~%float64[3] source_vect       # source vector[3]  ~%int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) ~%int8       ref               # DR_BASE(0), DR_WORLD(2), user coord(101~~200)~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AlignAxis2-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'target_vect) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'source_vect) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AlignAxis2-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AlignAxis2-request
    (cl:cons ':target_vect (target_vect msg))
    (cl:cons ':source_vect (source_vect msg))
    (cl:cons ':axis (axis msg))
    (cl:cons ':ref (ref msg))
))
;//! \htmlinclude AlignAxis2-response.msg.html

(cl:defclass <AlignAxis2-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AlignAxis2-response (<AlignAxis2-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AlignAxis2-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AlignAxis2-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<AlignAxis2-response> is deprecated: use dsr_msgs-srv:AlignAxis2-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <AlignAxis2-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AlignAxis2-response>) ostream)
  "Serializes a message object of type '<AlignAxis2-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AlignAxis2-response>) istream)
  "Deserializes a message object of type '<AlignAxis2-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AlignAxis2-response>)))
  "Returns string type for a service object of type '<AlignAxis2-response>"
  "dsr_msgs/AlignAxis2Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AlignAxis2-response)))
  "Returns string type for a service object of type 'AlignAxis2-response"
  "dsr_msgs/AlignAxis2Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AlignAxis2-response>)))
  "Returns md5sum for a message object of type '<AlignAxis2-response>"
  "789e8de44f005548a96c6375678d766c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AlignAxis2-response)))
  "Returns md5sum for a message object of type 'AlignAxis2-response"
  "789e8de44f005548a96c6375678d766c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AlignAxis2-response>)))
  "Returns full string definition for message of type '<AlignAxis2-response>"
  (cl:format cl:nil "bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AlignAxis2-response)))
  "Returns full string definition for message of type 'AlignAxis2-response"
  (cl:format cl:nil "bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AlignAxis2-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AlignAxis2-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AlignAxis2-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AlignAxis2)))
  'AlignAxis2-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AlignAxis2)))
  'AlignAxis2-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AlignAxis2)))
  "Returns string type for a service object of type '<AlignAxis2>"
  "dsr_msgs/AlignAxis2")