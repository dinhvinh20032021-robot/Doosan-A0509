; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetDesiredPosx-request.msg.html

(cl:defclass <GetDesiredPosx-request> (roslisp-msg-protocol:ros-message)
  ((ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetDesiredPosx-request (<GetDesiredPosx-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDesiredPosx-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDesiredPosx-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetDesiredPosx-request> is deprecated: use dsr_msgs-srv:GetDesiredPosx-request instead.")))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <GetDesiredPosx-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDesiredPosx-request>) ostream)
  "Serializes a message object of type '<GetDesiredPosx-request>"
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDesiredPosx-request>) istream)
  "Deserializes a message object of type '<GetDesiredPosx-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDesiredPosx-request>)))
  "Returns string type for a service object of type '<GetDesiredPosx-request>"
  "dsr_msgs/GetDesiredPosxRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDesiredPosx-request)))
  "Returns string type for a service object of type 'GetDesiredPosx-request"
  "dsr_msgs/GetDesiredPosxRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDesiredPosx-request>)))
  "Returns md5sum for a message object of type '<GetDesiredPosx-request>"
  "88547bdd1d2b717be93c04de7f32b5ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDesiredPosx-request)))
  "Returns md5sum for a message object of type 'GetDesiredPosx-request"
  "88547bdd1d2b717be93c04de7f32b5ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDesiredPosx-request>)))
  "Returns full string definition for message of type '<GetDesiredPosx-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_desired_posx(ref)  ~%#____________________________________________________________________________________________~%# This service returns the target pose of the current tool. The pose is based on the ref coordinate.~%~%int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2), user coord(101~~200)~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDesiredPosx-request)))
  "Returns full string definition for message of type 'GetDesiredPosx-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_desired_posx(ref)  ~%#____________________________________________________________________________________________~%# This service returns the target pose of the current tool. The pose is based on the ref coordinate.~%~%int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2), user coord(101~~200)~%                             # <ref is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDesiredPosx-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDesiredPosx-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDesiredPosx-request
    (cl:cons ':ref (ref msg))
))
;//! \htmlinclude GetDesiredPosx-response.msg.html

(cl:defclass <GetDesiredPosx-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetDesiredPosx-response (<GetDesiredPosx-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDesiredPosx-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDesiredPosx-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetDesiredPosx-response> is deprecated: use dsr_msgs-srv:GetDesiredPosx-response instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <GetDesiredPosx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:pos-val is deprecated.  Use dsr_msgs-srv:pos instead.")
  (pos m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetDesiredPosx-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDesiredPosx-response>) ostream)
  "Serializes a message object of type '<GetDesiredPosx-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDesiredPosx-response>) istream)
  "Deserializes a message object of type '<GetDesiredPosx-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDesiredPosx-response>)))
  "Returns string type for a service object of type '<GetDesiredPosx-response>"
  "dsr_msgs/GetDesiredPosxResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDesiredPosx-response)))
  "Returns string type for a service object of type 'GetDesiredPosx-response"
  "dsr_msgs/GetDesiredPosxResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDesiredPosx-response>)))
  "Returns md5sum for a message object of type '<GetDesiredPosx-response>"
  "88547bdd1d2b717be93c04de7f32b5ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDesiredPosx-response)))
  "Returns md5sum for a message object of type 'GetDesiredPosx-response"
  "88547bdd1d2b717be93c04de7f32b5ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDesiredPosx-response>)))
  "Returns full string definition for message of type '<GetDesiredPosx-response>"
  (cl:format cl:nil "float64[6] pos               # task pos(posx)~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDesiredPosx-response)))
  "Returns full string definition for message of type 'GetDesiredPosx-response"
  (cl:format cl:nil "float64[6] pos               # task pos(posx)~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDesiredPosx-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDesiredPosx-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDesiredPosx-response
    (cl:cons ':pos (pos msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetDesiredPosx)))
  'GetDesiredPosx-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetDesiredPosx)))
  'GetDesiredPosx-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDesiredPosx)))
  "Returns string type for a service object of type '<GetDesiredPosx>"
  "dsr_msgs/GetDesiredPosx")