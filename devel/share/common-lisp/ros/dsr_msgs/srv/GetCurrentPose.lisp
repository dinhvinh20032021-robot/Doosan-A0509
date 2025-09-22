; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetCurrentPose-request.msg.html

(cl:defclass <GetCurrentPose-request> (roslisp-msg-protocol:ros-message)
  ((space_type
    :reader space_type
    :initarg :space_type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetCurrentPose-request (<GetCurrentPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCurrentPose-request> is deprecated: use dsr_msgs-srv:GetCurrentPose-request instead.")))

(cl:ensure-generic-function 'space_type-val :lambda-list '(m))
(cl:defmethod space_type-val ((m <GetCurrentPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:space_type-val is deprecated.  Use dsr_msgs-srv:space_type instead.")
  (space_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentPose-request>) ostream)
  "Serializes a message object of type '<GetCurrentPose-request>"
  (cl:let* ((signed (cl:slot-value msg 'space_type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentPose-request>) istream)
  "Deserializes a message object of type '<GetCurrentPose-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'space_type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentPose-request>)))
  "Returns string type for a service object of type '<GetCurrentPose-request>"
  "dsr_msgs/GetCurrentPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentPose-request)))
  "Returns string type for a service object of type 'GetCurrentPose-request"
  "dsr_msgs/GetCurrentPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentPose-request>)))
  "Returns md5sum for a message object of type '<GetCurrentPose-request>"
  "7d90ac1bbfd6c3d94c5e9258853f72a9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentPose-request)))
  "Returns md5sum for a message object of type 'GetCurrentPose-request"
  "7d90ac1bbfd6c3d94c5e9258853f72a9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentPose-request>)))
  "Returns full string definition for message of type '<GetCurrentPose-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_current_pose~%#____________________________________________________________________________________________~%~%int8 space_type # 0=ROBOT_SPACE_JOINT, 1=ROBOT_SPACE_TASK~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentPose-request)))
  "Returns full string definition for message of type 'GetCurrentPose-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_current_pose~%#____________________________________________________________________________________________~%~%int8 space_type # 0=ROBOT_SPACE_JOINT, 1=ROBOT_SPACE_TASK~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentPose-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentPose-request
    (cl:cons ':space_type (space_type msg))
))
;//! \htmlinclude GetCurrentPose-response.msg.html

(cl:defclass <GetCurrentPose-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetCurrentPose-response (<GetCurrentPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCurrentPose-response> is deprecated: use dsr_msgs-srv:GetCurrentPose-response instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <GetCurrentPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:pos-val is deprecated.  Use dsr_msgs-srv:pos instead.")
  (pos m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetCurrentPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentPose-response>) ostream)
  "Serializes a message object of type '<GetCurrentPose-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentPose-response>) istream)
  "Deserializes a message object of type '<GetCurrentPose-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentPose-response>)))
  "Returns string type for a service object of type '<GetCurrentPose-response>"
  "dsr_msgs/GetCurrentPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentPose-response)))
  "Returns string type for a service object of type 'GetCurrentPose-response"
  "dsr_msgs/GetCurrentPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentPose-response>)))
  "Returns md5sum for a message object of type '<GetCurrentPose-response>"
  "7d90ac1bbfd6c3d94c5e9258853f72a9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentPose-response)))
  "Returns md5sum for a message object of type 'GetCurrentPose-response"
  "7d90ac1bbfd6c3d94c5e9258853f72a9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentPose-response>)))
  "Returns full string definition for message of type '<GetCurrentPose-response>"
  (cl:format cl:nil "float64[6] pos~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentPose-response)))
  "Returns full string definition for message of type 'GetCurrentPose-response"
  (cl:format cl:nil "float64[6] pos~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentPose-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentPose-response
    (cl:cons ':pos (pos msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCurrentPose)))
  'GetCurrentPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCurrentPose)))
  'GetCurrentPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentPose)))
  "Returns string type for a service object of type '<GetCurrentPose>"
  "dsr_msgs/GetCurrentPose")