; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetOrientationError-request.msg.html

(cl:defclass <GetOrientationError-request> (roslisp-msg-protocol:ros-message)
  ((xd
    :reader xd
    :initarg :xd
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (xc
    :reader xc
    :initarg :xc
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (axis
    :reader axis
    :initarg :axis
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetOrientationError-request (<GetOrientationError-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetOrientationError-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetOrientationError-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetOrientationError-request> is deprecated: use dsr_msgs-srv:GetOrientationError-request instead.")))

(cl:ensure-generic-function 'xd-val :lambda-list '(m))
(cl:defmethod xd-val ((m <GetOrientationError-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:xd-val is deprecated.  Use dsr_msgs-srv:xd instead.")
  (xd m))

(cl:ensure-generic-function 'xc-val :lambda-list '(m))
(cl:defmethod xc-val ((m <GetOrientationError-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:xc-val is deprecated.  Use dsr_msgs-srv:xc instead.")
  (xc m))

(cl:ensure-generic-function 'axis-val :lambda-list '(m))
(cl:defmethod axis-val ((m <GetOrientationError-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:axis-val is deprecated.  Use dsr_msgs-srv:axis instead.")
  (axis m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetOrientationError-request>) ostream)
  "Serializes a message object of type '<GetOrientationError-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'xd))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'xc))
  (cl:let* ((signed (cl:slot-value msg 'axis)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetOrientationError-request>) istream)
  "Deserializes a message object of type '<GetOrientationError-request>"
  (cl:setf (cl:slot-value msg 'xd) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'xd)))
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
  (cl:setf (cl:slot-value msg 'xc) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'xc)))
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
      (cl:setf (cl:slot-value msg 'axis) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetOrientationError-request>)))
  "Returns string type for a service object of type '<GetOrientationError-request>"
  "dsr_msgs/GetOrientationErrorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOrientationError-request)))
  "Returns string type for a service object of type 'GetOrientationError-request"
  "dsr_msgs/GetOrientationErrorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetOrientationError-request>)))
  "Returns md5sum for a message object of type '<GetOrientationError-request>"
  "83afbc83b0df709d838d4a917c3c9c05")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetOrientationError-request)))
  "Returns md5sum for a message object of type 'GetOrientationError-request"
  "83afbc83b0df709d838d4a917c3c9c05")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetOrientationError-request>)))
  "Returns full string definition for message of type '<GetOrientationError-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_orientation_error  ~%#____________________________________________________________________________________________~%# This service returns the orientation error value between the arbitrary poses xd and xc of the axis.~%~%float64[6] xd                # task pos(posx)  ~%float64[6] xc                # task pos(posx)  ~%int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetOrientationError-request)))
  "Returns full string definition for message of type 'GetOrientationError-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_orientation_error  ~%#____________________________________________________________________________________________~%# This service returns the orientation error value between the arbitrary poses xd and xc of the axis.~%~%float64[6] xd                # task pos(posx)  ~%float64[6] xc                # task pos(posx)  ~%int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetOrientationError-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'xd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'xc) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetOrientationError-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetOrientationError-request
    (cl:cons ':xd (xd msg))
    (cl:cons ':xc (xc msg))
    (cl:cons ':axis (axis msg))
))
;//! \htmlinclude GetOrientationError-response.msg.html

(cl:defclass <GetOrientationError-response> (roslisp-msg-protocol:ros-message)
  ((ori_error
    :reader ori_error
    :initarg :ori_error
    :type cl:float
    :initform 0.0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetOrientationError-response (<GetOrientationError-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetOrientationError-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetOrientationError-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetOrientationError-response> is deprecated: use dsr_msgs-srv:GetOrientationError-response instead.")))

(cl:ensure-generic-function 'ori_error-val :lambda-list '(m))
(cl:defmethod ori_error-val ((m <GetOrientationError-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ori_error-val is deprecated.  Use dsr_msgs-srv:ori_error instead.")
  (ori_error m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetOrientationError-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetOrientationError-response>) ostream)
  "Serializes a message object of type '<GetOrientationError-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ori_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetOrientationError-response>) istream)
  "Deserializes a message object of type '<GetOrientationError-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ori_error) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetOrientationError-response>)))
  "Returns string type for a service object of type '<GetOrientationError-response>"
  "dsr_msgs/GetOrientationErrorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOrientationError-response)))
  "Returns string type for a service object of type 'GetOrientationError-response"
  "dsr_msgs/GetOrientationErrorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetOrientationError-response>)))
  "Returns md5sum for a message object of type '<GetOrientationError-response>"
  "83afbc83b0df709d838d4a917c3c9c05")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetOrientationError-response)))
  "Returns md5sum for a message object of type 'GetOrientationError-response"
  "83afbc83b0df709d838d4a917c3c9c05")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetOrientationError-response>)))
  "Returns full string definition for message of type '<GetOrientationError-response>"
  (cl:format cl:nil "float32    ori_error         # orientation error  ~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetOrientationError-response)))
  "Returns full string definition for message of type 'GetOrientationError-response"
  (cl:format cl:nil "float32    ori_error         # orientation error  ~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetOrientationError-response>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetOrientationError-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetOrientationError-response
    (cl:cons ':ori_error (ori_error msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetOrientationError)))
  'GetOrientationError-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetOrientationError)))
  'GetOrientationError-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOrientationError)))
  "Returns string type for a service object of type '<GetOrientationError>"
  "dsr_msgs/GetOrientationError")