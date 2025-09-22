; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetToolForce-request.msg.html

(cl:defclass <GetToolForce-request> (roslisp-msg-protocol:ros-message)
  ((ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetToolForce-request (<GetToolForce-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetToolForce-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetToolForce-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetToolForce-request> is deprecated: use dsr_msgs-srv:GetToolForce-request instead.")))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <GetToolForce-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetToolForce-request>) ostream)
  "Serializes a message object of type '<GetToolForce-request>"
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetToolForce-request>) istream)
  "Deserializes a message object of type '<GetToolForce-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetToolForce-request>)))
  "Returns string type for a service object of type '<GetToolForce-request>"
  "dsr_msgs/GetToolForceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetToolForce-request)))
  "Returns string type for a service object of type 'GetToolForce-request"
  "dsr_msgs/GetToolForceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetToolForce-request>)))
  "Returns md5sum for a message object of type '<GetToolForce-request>"
  "9590f477df24281be1fec298a858c77f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetToolForce-request)))
  "Returns md5sum for a message object of type 'GetToolForce-request"
  "9590f477df24281be1fec298a858c77f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetToolForce-request>)))
  "Returns full string definition for message of type '<GetToolForce-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_tool_force(ref)~%# returns the external force applied to the current tool~%#____________________________________________________________________________________________~%# This service returns the external force applied to the current tool based on the ref coordinate. ~%# The force is based on the base coordinate while the moment is based on the tool coordinate.~%~%int8       ref               # DR_BASE(0), DR_WORLD(2)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetToolForce-request)))
  "Returns full string definition for message of type 'GetToolForce-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_tool_force(ref)~%# returns the external force applied to the current tool~%#____________________________________________________________________________________________~%# This service returns the external force applied to the current tool based on the ref coordinate. ~%# The force is based on the base coordinate while the moment is based on the tool coordinate.~%~%int8       ref               # DR_BASE(0), DR_WORLD(2)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetToolForce-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetToolForce-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetToolForce-request
    (cl:cons ':ref (ref msg))
))
;//! \htmlinclude GetToolForce-response.msg.html

(cl:defclass <GetToolForce-response> (roslisp-msg-protocol:ros-message)
  ((tool_force
    :reader tool_force
    :initarg :tool_force
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetToolForce-response (<GetToolForce-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetToolForce-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetToolForce-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetToolForce-response> is deprecated: use dsr_msgs-srv:GetToolForce-response instead.")))

(cl:ensure-generic-function 'tool_force-val :lambda-list '(m))
(cl:defmethod tool_force-val ((m <GetToolForce-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:tool_force-val is deprecated.  Use dsr_msgs-srv:tool_force instead.")
  (tool_force m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetToolForce-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetToolForce-response>) ostream)
  "Serializes a message object of type '<GetToolForce-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'tool_force))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetToolForce-response>) istream)
  "Deserializes a message object of type '<GetToolForce-response>"
  (cl:setf (cl:slot-value msg 'tool_force) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'tool_force)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetToolForce-response>)))
  "Returns string type for a service object of type '<GetToolForce-response>"
  "dsr_msgs/GetToolForceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetToolForce-response)))
  "Returns string type for a service object of type 'GetToolForce-response"
  "dsr_msgs/GetToolForceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetToolForce-response>)))
  "Returns md5sum for a message object of type '<GetToolForce-response>"
  "9590f477df24281be1fec298a858c77f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetToolForce-response)))
  "Returns md5sum for a message object of type 'GetToolForce-response"
  "9590f477df24281be1fec298a858c77f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetToolForce-response>)))
  "Returns full string definition for message of type '<GetToolForce-response>"
  (cl:format cl:nil "float64[6] tool_force        # External force applied to the tool~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetToolForce-response)))
  "Returns full string definition for message of type 'GetToolForce-response"
  (cl:format cl:nil "float64[6] tool_force        # External force applied to the tool~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetToolForce-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'tool_force) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetToolForce-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetToolForce-response
    (cl:cons ':tool_force (tool_force msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetToolForce)))
  'GetToolForce-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetToolForce)))
  'GetToolForce-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetToolForce)))
  "Returns string type for a service object of type '<GetToolForce>"
  "dsr_msgs/GetToolForce")