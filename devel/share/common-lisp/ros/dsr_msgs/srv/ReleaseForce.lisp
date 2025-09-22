; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude ReleaseForce-request.msg.html

(cl:defclass <ReleaseForce-request> (roslisp-msg-protocol:ros-message)
  ((time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0))
)

(cl:defclass ReleaseForce-request (<ReleaseForce-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReleaseForce-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReleaseForce-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ReleaseForce-request> is deprecated: use dsr_msgs-srv:ReleaseForce-request instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <ReleaseForce-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:time-val is deprecated.  Use dsr_msgs-srv:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReleaseForce-request>) ostream)
  "Serializes a message object of type '<ReleaseForce-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReleaseForce-request>) istream)
  "Deserializes a message object of type '<ReleaseForce-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReleaseForce-request>)))
  "Returns string type for a service object of type '<ReleaseForce-request>"
  "dsr_msgs/ReleaseForceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReleaseForce-request)))
  "Returns string type for a service object of type 'ReleaseForce-request"
  "dsr_msgs/ReleaseForceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReleaseForce-request>)))
  "Returns md5sum for a message object of type '<ReleaseForce-request>"
  "54a0fd05cf7c093a898987563c9ea327")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReleaseForce-request)))
  "Returns md5sum for a message object of type 'ReleaseForce-request"
  "54a0fd05cf7c093a898987563c9ea327")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReleaseForce-request>)))
  "Returns full string definition for message of type '<ReleaseForce-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# release_force  ~%#____________________________________________________________________________________________~%~%float64    time # 0          # Time needed to reduce the force (0 ~~ 1.0) ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReleaseForce-request)))
  "Returns full string definition for message of type 'ReleaseForce-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# release_force  ~%#____________________________________________________________________________________________~%~%float64    time # 0          # Time needed to reduce the force (0 ~~ 1.0) ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReleaseForce-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReleaseForce-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReleaseForce-request
    (cl:cons ':time (time msg))
))
;//! \htmlinclude ReleaseForce-response.msg.html

(cl:defclass <ReleaseForce-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ReleaseForce-response (<ReleaseForce-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReleaseForce-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReleaseForce-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ReleaseForce-response> is deprecated: use dsr_msgs-srv:ReleaseForce-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ReleaseForce-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReleaseForce-response>) ostream)
  "Serializes a message object of type '<ReleaseForce-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReleaseForce-response>) istream)
  "Deserializes a message object of type '<ReleaseForce-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReleaseForce-response>)))
  "Returns string type for a service object of type '<ReleaseForce-response>"
  "dsr_msgs/ReleaseForceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReleaseForce-response)))
  "Returns string type for a service object of type 'ReleaseForce-response"
  "dsr_msgs/ReleaseForceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReleaseForce-response>)))
  "Returns md5sum for a message object of type '<ReleaseForce-response>"
  "54a0fd05cf7c093a898987563c9ea327")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReleaseForce-response)))
  "Returns md5sum for a message object of type 'ReleaseForce-response"
  "54a0fd05cf7c093a898987563c9ea327")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReleaseForce-response>)))
  "Returns full string definition for message of type '<ReleaseForce-response>"
  (cl:format cl:nil "bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReleaseForce-response)))
  "Returns full string definition for message of type 'ReleaseForce-response"
  (cl:format cl:nil "bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReleaseForce-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReleaseForce-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReleaseForce-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReleaseForce)))
  'ReleaseForce-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReleaseForce)))
  'ReleaseForce-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReleaseForce)))
  "Returns string type for a service object of type '<ReleaseForce>"
  "dsr_msgs/ReleaseForce")