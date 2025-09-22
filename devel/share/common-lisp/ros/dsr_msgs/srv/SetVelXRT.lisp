; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetVelXRT-request.msg.html

(cl:defclass <SetVelXRT-request> (roslisp-msg-protocol:ros-message)
  ((trans
    :reader trans
    :initarg :trans
    :type cl:float
    :initform 0.0)
   (rotation
    :reader rotation
    :initarg :rotation
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetVelXRT-request (<SetVelXRT-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVelXRT-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVelXRT-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetVelXRT-request> is deprecated: use dsr_msgs-srv:SetVelXRT-request instead.")))

(cl:ensure-generic-function 'trans-val :lambda-list '(m))
(cl:defmethod trans-val ((m <SetVelXRT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:trans-val is deprecated.  Use dsr_msgs-srv:trans instead.")
  (trans m))

(cl:ensure-generic-function 'rotation-val :lambda-list '(m))
(cl:defmethod rotation-val ((m <SetVelXRT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:rotation-val is deprecated.  Use dsr_msgs-srv:rotation instead.")
  (rotation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVelXRT-request>) ostream)
  "Serializes a message object of type '<SetVelXRT-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'trans))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rotation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVelXRT-request>) istream)
  "Deserializes a message object of type '<SetVelXRT-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'trans) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotation) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVelXRT-request>)))
  "Returns string type for a service object of type '<SetVelXRT-request>"
  "dsr_msgs/SetVelXRTRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVelXRT-request)))
  "Returns string type for a service object of type 'SetVelXRT-request"
  "dsr_msgs/SetVelXRTRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVelXRT-request>)))
  "Returns md5sum for a message object of type '<SetVelXRT-request>"
  "761d85a527beb96265a49a9c48c83232")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVelXRT-request)))
  "Returns md5sum for a message object of type 'SetVelXRT-request"
  "761d85a527beb96265a49a9c48c83232")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVelXRT-request>)))
  "Returns full string definition for message of type '<SetVelXRT-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_velx_rt~%#____________________________________________________________________________________________~%float64    trans~%float64    rotation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVelXRT-request)))
  "Returns full string definition for message of type 'SetVelXRT-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_velx_rt~%#____________________________________________________________________________________________~%float64    trans~%float64    rotation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVelXRT-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVelXRT-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVelXRT-request
    (cl:cons ':trans (trans msg))
    (cl:cons ':rotation (rotation msg))
))
;//! \htmlinclude SetVelXRT-response.msg.html

(cl:defclass <SetVelXRT-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetVelXRT-response (<SetVelXRT-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVelXRT-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVelXRT-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetVelXRT-response> is deprecated: use dsr_msgs-srv:SetVelXRT-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetVelXRT-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVelXRT-response>) ostream)
  "Serializes a message object of type '<SetVelXRT-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVelXRT-response>) istream)
  "Deserializes a message object of type '<SetVelXRT-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVelXRT-response>)))
  "Returns string type for a service object of type '<SetVelXRT-response>"
  "dsr_msgs/SetVelXRTResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVelXRT-response)))
  "Returns string type for a service object of type 'SetVelXRT-response"
  "dsr_msgs/SetVelXRTResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVelXRT-response>)))
  "Returns md5sum for a message object of type '<SetVelXRT-response>"
  "761d85a527beb96265a49a9c48c83232")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVelXRT-response)))
  "Returns md5sum for a message object of type 'SetVelXRT-response"
  "761d85a527beb96265a49a9c48c83232")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVelXRT-response>)))
  "Returns full string definition for message of type '<SetVelXRT-response>"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVelXRT-response)))
  "Returns full string definition for message of type 'SetVelXRT-response"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVelXRT-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVelXRT-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVelXRT-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetVelXRT)))
  'SetVelXRT-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetVelXRT)))
  'SetVelXRT-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVelXRT)))
  "Returns string type for a service object of type '<SetVelXRT>"
  "dsr_msgs/SetVelXRT")