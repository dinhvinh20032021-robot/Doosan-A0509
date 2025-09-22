; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetCurrentToolFlangePosx-request.msg.html

(cl:defclass <GetCurrentToolFlangePosx-request> (roslisp-msg-protocol:ros-message)
  ((ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetCurrentToolFlangePosx-request (<GetCurrentToolFlangePosx-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentToolFlangePosx-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentToolFlangePosx-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCurrentToolFlangePosx-request> is deprecated: use dsr_msgs-srv:GetCurrentToolFlangePosx-request instead.")))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <GetCurrentToolFlangePosx-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentToolFlangePosx-request>) ostream)
  "Serializes a message object of type '<GetCurrentToolFlangePosx-request>"
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentToolFlangePosx-request>) istream)
  "Deserializes a message object of type '<GetCurrentToolFlangePosx-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentToolFlangePosx-request>)))
  "Returns string type for a service object of type '<GetCurrentToolFlangePosx-request>"
  "dsr_msgs/GetCurrentToolFlangePosxRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentToolFlangePosx-request)))
  "Returns string type for a service object of type 'GetCurrentToolFlangePosx-request"
  "dsr_msgs/GetCurrentToolFlangePosxRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentToolFlangePosx-request>)))
  "Returns md5sum for a message object of type '<GetCurrentToolFlangePosx-request>"
  "88547bdd1d2b717be93c04de7f32b5ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentToolFlangePosx-request)))
  "Returns md5sum for a message object of type 'GetCurrentToolFlangePosx-request"
  "88547bdd1d2b717be93c04de7f32b5ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentToolFlangePosx-request>)))
  "Returns full string definition for message of type '<GetCurrentToolFlangePosx-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_current_tool_flange_posx()  ~%#____________________________________________________________________________________________~%# This service returns the pose of the current tool flange based on the ref coordinate. In other words, it means the return to tcp=(0,0,0,0,0,0).~%~%int8        ref               # DR_BASE(0), DR_WORLD(2)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentToolFlangePosx-request)))
  "Returns full string definition for message of type 'GetCurrentToolFlangePosx-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_current_tool_flange_posx()  ~%#____________________________________________________________________________________________~%# This service returns the pose of the current tool flange based on the ref coordinate. In other words, it means the return to tcp=(0,0,0,0,0,0).~%~%int8        ref               # DR_BASE(0), DR_WORLD(2)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentToolFlangePosx-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentToolFlangePosx-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentToolFlangePosx-request
    (cl:cons ':ref (ref msg))
))
;//! \htmlinclude GetCurrentToolFlangePosx-response.msg.html

(cl:defclass <GetCurrentToolFlangePosx-response> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetCurrentToolFlangePosx-response (<GetCurrentToolFlangePosx-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentToolFlangePosx-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentToolFlangePosx-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCurrentToolFlangePosx-response> is deprecated: use dsr_msgs-srv:GetCurrentToolFlangePosx-response instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <GetCurrentToolFlangePosx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:pos-val is deprecated.  Use dsr_msgs-srv:pos instead.")
  (pos m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetCurrentToolFlangePosx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentToolFlangePosx-response>) ostream)
  "Serializes a message object of type '<GetCurrentToolFlangePosx-response>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentToolFlangePosx-response>) istream)
  "Deserializes a message object of type '<GetCurrentToolFlangePosx-response>"
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
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentToolFlangePosx-response>)))
  "Returns string type for a service object of type '<GetCurrentToolFlangePosx-response>"
  "dsr_msgs/GetCurrentToolFlangePosxResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentToolFlangePosx-response)))
  "Returns string type for a service object of type 'GetCurrentToolFlangePosx-response"
  "dsr_msgs/GetCurrentToolFlangePosxResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentToolFlangePosx-response>)))
  "Returns md5sum for a message object of type '<GetCurrentToolFlangePosx-response>"
  "88547bdd1d2b717be93c04de7f32b5ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentToolFlangePosx-response)))
  "Returns md5sum for a message object of type 'GetCurrentToolFlangePosx-response"
  "88547bdd1d2b717be93c04de7f32b5ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentToolFlangePosx-response>)))
  "Returns full string definition for message of type '<GetCurrentToolFlangePosx-response>"
  (cl:format cl:nil "float64[6]  pos               # Pose of tool flange(posx) ~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentToolFlangePosx-response)))
  "Returns full string definition for message of type 'GetCurrentToolFlangePosx-response"
  (cl:format cl:nil "float64[6]  pos               # Pose of tool flange(posx) ~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentToolFlangePosx-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentToolFlangePosx-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentToolFlangePosx-response
    (cl:cons ':pos (pos msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCurrentToolFlangePosx)))
  'GetCurrentToolFlangePosx-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCurrentToolFlangePosx)))
  'GetCurrentToolFlangePosx-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentToolFlangePosx)))
  "Returns string type for a service object of type '<GetCurrentToolFlangePosx>"
  "dsr_msgs/GetCurrentToolFlangePosx")