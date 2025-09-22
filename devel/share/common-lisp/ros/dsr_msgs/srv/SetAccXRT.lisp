; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetAccXRT-request.msg.html

(cl:defclass <SetAccXRT-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetAccXRT-request (<SetAccXRT-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAccXRT-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAccXRT-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetAccXRT-request> is deprecated: use dsr_msgs-srv:SetAccXRT-request instead.")))

(cl:ensure-generic-function 'trans-val :lambda-list '(m))
(cl:defmethod trans-val ((m <SetAccXRT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:trans-val is deprecated.  Use dsr_msgs-srv:trans instead.")
  (trans m))

(cl:ensure-generic-function 'rotation-val :lambda-list '(m))
(cl:defmethod rotation-val ((m <SetAccXRT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:rotation-val is deprecated.  Use dsr_msgs-srv:rotation instead.")
  (rotation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAccXRT-request>) ostream)
  "Serializes a message object of type '<SetAccXRT-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAccXRT-request>) istream)
  "Deserializes a message object of type '<SetAccXRT-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAccXRT-request>)))
  "Returns string type for a service object of type '<SetAccXRT-request>"
  "dsr_msgs/SetAccXRTRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAccXRT-request)))
  "Returns string type for a service object of type 'SetAccXRT-request"
  "dsr_msgs/SetAccXRTRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAccXRT-request>)))
  "Returns md5sum for a message object of type '<SetAccXRT-request>"
  "761d85a527beb96265a49a9c48c83232")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAccXRT-request)))
  "Returns md5sum for a message object of type 'SetAccXRT-request"
  "761d85a527beb96265a49a9c48c83232")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAccXRT-request>)))
  "Returns full string definition for message of type '<SetAccXRT-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_accx_rt~%#____________________________________________________________________________________________~%float64    trans~%float64    rotation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAccXRT-request)))
  "Returns full string definition for message of type 'SetAccXRT-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_accx_rt~%#____________________________________________________________________________________________~%float64    trans~%float64    rotation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAccXRT-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAccXRT-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAccXRT-request
    (cl:cons ':trans (trans msg))
    (cl:cons ':rotation (rotation msg))
))
;//! \htmlinclude SetAccXRT-response.msg.html

(cl:defclass <SetAccXRT-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetAccXRT-response (<SetAccXRT-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAccXRT-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAccXRT-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetAccXRT-response> is deprecated: use dsr_msgs-srv:SetAccXRT-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetAccXRT-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAccXRT-response>) ostream)
  "Serializes a message object of type '<SetAccXRT-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAccXRT-response>) istream)
  "Deserializes a message object of type '<SetAccXRT-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAccXRT-response>)))
  "Returns string type for a service object of type '<SetAccXRT-response>"
  "dsr_msgs/SetAccXRTResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAccXRT-response)))
  "Returns string type for a service object of type 'SetAccXRT-response"
  "dsr_msgs/SetAccXRTResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAccXRT-response>)))
  "Returns md5sum for a message object of type '<SetAccXRT-response>"
  "761d85a527beb96265a49a9c48c83232")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAccXRT-response)))
  "Returns md5sum for a message object of type 'SetAccXRT-response"
  "761d85a527beb96265a49a9c48c83232")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAccXRT-response>)))
  "Returns full string definition for message of type '<SetAccXRT-response>"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAccXRT-response)))
  "Returns full string definition for message of type 'SetAccXRT-response"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAccXRT-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAccXRT-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAccXRT-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetAccXRT)))
  'SetAccXRT-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetAccXRT)))
  'SetAccXRT-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAccXRT)))
  "Returns string type for a service object of type '<SetAccXRT>"
  "dsr_msgs/SetAccXRT")