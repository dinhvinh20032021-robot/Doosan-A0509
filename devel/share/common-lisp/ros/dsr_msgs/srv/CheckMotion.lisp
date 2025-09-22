; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude CheckMotion-request.msg.html

(cl:defclass <CheckMotion-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CheckMotion-request (<CheckMotion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckMotion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckMotion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<CheckMotion-request> is deprecated: use dsr_msgs-srv:CheckMotion-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckMotion-request>) ostream)
  "Serializes a message object of type '<CheckMotion-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckMotion-request>) istream)
  "Deserializes a message object of type '<CheckMotion-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckMotion-request>)))
  "Returns string type for a service object of type '<CheckMotion-request>"
  "dsr_msgs/CheckMotionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckMotion-request)))
  "Returns string type for a service object of type 'CheckMotion-request"
  "dsr_msgs/CheckMotionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckMotion-request>)))
  "Returns md5sum for a message object of type '<CheckMotion-request>"
  "f2991389d99138c3293e56c6922ac005")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckMotion-request)))
  "Returns md5sum for a message object of type 'CheckMotion-request"
  "f2991389d99138c3293e56c6922ac005")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckMotion-request>)))
  "Returns full string definition for message of type '<CheckMotion-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# check_motion~%# return status of the currently active motion.~%# Homing is performed by moving to the joint motion to the mechanical or user defined home position.~%# According to the input parameter [target], it moves to the mechanical home defined in the system or the home set by the user.~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckMotion-request)))
  "Returns full string definition for message of type 'CheckMotion-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# check_motion~%# return status of the currently active motion.~%# Homing is performed by moving to the joint motion to the mechanical or user defined home position.~%# According to the input parameter [target], it moves to the mechanical home defined in the system or the home set by the user.~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckMotion-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckMotion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckMotion-request
))
;//! \htmlinclude CheckMotion-response.msg.html

(cl:defclass <CheckMotion-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CheckMotion-response (<CheckMotion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckMotion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckMotion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<CheckMotion-response> is deprecated: use dsr_msgs-srv:CheckMotion-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <CheckMotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:status-val is deprecated.  Use dsr_msgs-srv:status instead.")
  (status m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <CheckMotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckMotion-response>) ostream)
  "Serializes a message object of type '<CheckMotion-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckMotion-response>) istream)
  "Deserializes a message object of type '<CheckMotion-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckMotion-response>)))
  "Returns string type for a service object of type '<CheckMotion-response>"
  "dsr_msgs/CheckMotionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckMotion-response)))
  "Returns string type for a service object of type 'CheckMotion-response"
  "dsr_msgs/CheckMotionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckMotion-response>)))
  "Returns md5sum for a message object of type '<CheckMotion-response>"
  "f2991389d99138c3293e56c6922ac005")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckMotion-response)))
  "Returns md5sum for a message object of type 'CheckMotion-response"
  "f2991389d99138c3293e56c6922ac005")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckMotion-response>)))
  "Returns full string definition for message of type '<CheckMotion-response>"
  (cl:format cl:nil "int8       status          # DR_STATE_IDLE(0) : no motion in action~%                           # DR_STATE_INIT(1) : motion being calculated~%                           # DR_STATE_BUSY(2) : motion in operation~%bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckMotion-response)))
  "Returns full string definition for message of type 'CheckMotion-response"
  (cl:format cl:nil "int8       status          # DR_STATE_IDLE(0) : no motion in action~%                           # DR_STATE_INIT(1) : motion being calculated~%                           # DR_STATE_BUSY(2) : motion in operation~%bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckMotion-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckMotion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckMotion-response
    (cl:cons ':status (status msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CheckMotion)))
  'CheckMotion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CheckMotion)))
  'CheckMotion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckMotion)))
  "Returns string type for a service object of type '<CheckMotion>"
  "dsr_msgs/CheckMotion")