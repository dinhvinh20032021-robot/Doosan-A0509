; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude CoordTransform-request.msg.html

(cl:defclass <CoordTransform-request> (roslisp-msg-protocol:ros-message)
  ((pos_in
    :reader pos_in
    :initarg :pos_in
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (ref_in
    :reader ref_in
    :initarg :ref_in
    :type cl:fixnum
    :initform 0)
   (ref_out
    :reader ref_out
    :initarg :ref_out
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CoordTransform-request (<CoordTransform-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CoordTransform-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CoordTransform-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<CoordTransform-request> is deprecated: use dsr_msgs-srv:CoordTransform-request instead.")))

(cl:ensure-generic-function 'pos_in-val :lambda-list '(m))
(cl:defmethod pos_in-val ((m <CoordTransform-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:pos_in-val is deprecated.  Use dsr_msgs-srv:pos_in instead.")
  (pos_in m))

(cl:ensure-generic-function 'ref_in-val :lambda-list '(m))
(cl:defmethod ref_in-val ((m <CoordTransform-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref_in-val is deprecated.  Use dsr_msgs-srv:ref_in instead.")
  (ref_in m))

(cl:ensure-generic-function 'ref_out-val :lambda-list '(m))
(cl:defmethod ref_out-val ((m <CoordTransform-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref_out-val is deprecated.  Use dsr_msgs-srv:ref_out instead.")
  (ref_out m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CoordTransform-request>) ostream)
  "Serializes a message object of type '<CoordTransform-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'pos_in))
  (cl:let* ((signed (cl:slot-value msg 'ref_in)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ref_out)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CoordTransform-request>) istream)
  "Deserializes a message object of type '<CoordTransform-request>"
  (cl:setf (cl:slot-value msg 'pos_in) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'pos_in)))
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
      (cl:setf (cl:slot-value msg 'ref_in) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref_out) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CoordTransform-request>)))
  "Returns string type for a service object of type '<CoordTransform-request>"
  "dsr_msgs/CoordTransformRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CoordTransform-request)))
  "Returns string type for a service object of type 'CoordTransform-request"
  "dsr_msgs/CoordTransformRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CoordTransform-request>)))
  "Returns md5sum for a message object of type '<CoordTransform-request>"
  "0cb6558d600295c3a86e9d114e608d8a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CoordTransform-request)))
  "Returns md5sum for a message object of type 'CoordTransform-request"
  "0cb6558d600295c3a86e9d114e608d8a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CoordTransform-request>)))
  "Returns full string definition for message of type '<CoordTransform-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# coord_transform   ~%#____________________________________________________________________________________________~%~%float64[6] pos_in            # task pos(posx)  ~%int8       ref_in            # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~~200)~%                             # <ref is only available in M2.40 or later> ~%int8       ref_out           # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~~200) ~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CoordTransform-request)))
  "Returns full string definition for message of type 'CoordTransform-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# coord_transform   ~%#____________________________________________________________________________________________~%~%float64[6] pos_in            # task pos(posx)  ~%int8       ref_in            # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~~200)~%                             # <ref is only available in M2.40 or later> ~%int8       ref_out           # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~~200) ~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CoordTransform-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos_in) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CoordTransform-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CoordTransform-request
    (cl:cons ':pos_in (pos_in msg))
    (cl:cons ':ref_in (ref_in msg))
    (cl:cons ':ref_out (ref_out msg))
))
;//! \htmlinclude CoordTransform-response.msg.html

(cl:defclass <CoordTransform-response> (roslisp-msg-protocol:ros-message)
  ((conv_posx
    :reader conv_posx
    :initarg :conv_posx
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CoordTransform-response (<CoordTransform-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CoordTransform-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CoordTransform-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<CoordTransform-response> is deprecated: use dsr_msgs-srv:CoordTransform-response instead.")))

(cl:ensure-generic-function 'conv_posx-val :lambda-list '(m))
(cl:defmethod conv_posx-val ((m <CoordTransform-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:conv_posx-val is deprecated.  Use dsr_msgs-srv:conv_posx instead.")
  (conv_posx m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <CoordTransform-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CoordTransform-response>) ostream)
  "Serializes a message object of type '<CoordTransform-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'conv_posx))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CoordTransform-response>) istream)
  "Deserializes a message object of type '<CoordTransform-response>"
  (cl:setf (cl:slot-value msg 'conv_posx) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'conv_posx)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CoordTransform-response>)))
  "Returns string type for a service object of type '<CoordTransform-response>"
  "dsr_msgs/CoordTransformResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CoordTransform-response)))
  "Returns string type for a service object of type 'CoordTransform-response"
  "dsr_msgs/CoordTransformResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CoordTransform-response>)))
  "Returns md5sum for a message object of type '<CoordTransform-response>"
  "0cb6558d600295c3a86e9d114e608d8a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CoordTransform-response)))
  "Returns md5sum for a message object of type 'CoordTransform-response"
  "0cb6558d600295c3a86e9d114e608d8a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CoordTransform-response>)))
  "Returns full string definition for message of type '<CoordTransform-response>"
  (cl:format cl:nil "float64[6] conv_posx         # task pos(posx)~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CoordTransform-response)))
  "Returns full string definition for message of type 'CoordTransform-response"
  (cl:format cl:nil "float64[6] conv_posx         # task pos(posx)~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CoordTransform-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'conv_posx) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CoordTransform-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CoordTransform-response
    (cl:cons ':conv_posx (conv_posx msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CoordTransform)))
  'CoordTransform-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CoordTransform)))
  'CoordTransform-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CoordTransform)))
  "Returns string type for a service object of type '<CoordTransform>"
  "dsr_msgs/CoordTransform")