; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetWorkpieceWeight-request.msg.html

(cl:defclass <GetWorkpieceWeight-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetWorkpieceWeight-request (<GetWorkpieceWeight-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWorkpieceWeight-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWorkpieceWeight-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetWorkpieceWeight-request> is deprecated: use dsr_msgs-srv:GetWorkpieceWeight-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWorkpieceWeight-request>) ostream)
  "Serializes a message object of type '<GetWorkpieceWeight-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWorkpieceWeight-request>) istream)
  "Deserializes a message object of type '<GetWorkpieceWeight-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWorkpieceWeight-request>)))
  "Returns string type for a service object of type '<GetWorkpieceWeight-request>"
  "dsr_msgs/GetWorkpieceWeightRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWorkpieceWeight-request)))
  "Returns string type for a service object of type 'GetWorkpieceWeight-request"
  "dsr_msgs/GetWorkpieceWeightRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWorkpieceWeight-request>)))
  "Returns md5sum for a message object of type '<GetWorkpieceWeight-request>"
  "79fbe0854a3099a7d5008b26f92ad15a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWorkpieceWeight-request)))
  "Returns md5sum for a message object of type 'GetWorkpieceWeight-request"
  "79fbe0854a3099a7d5008b26f92ad15a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWorkpieceWeight-request>)))
  "Returns full string definition for message of type '<GetWorkpieceWeight-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_workpiece_weight~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWorkpieceWeight-request)))
  "Returns full string definition for message of type 'GetWorkpieceWeight-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_workpiece_weight~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWorkpieceWeight-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWorkpieceWeight-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWorkpieceWeight-request
))
;//! \htmlinclude GetWorkpieceWeight-response.msg.html

(cl:defclass <GetWorkpieceWeight-response> (roslisp-msg-protocol:ros-message)
  ((weight
    :reader weight
    :initarg :weight
    :type cl:float
    :initform 0.0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetWorkpieceWeight-response (<GetWorkpieceWeight-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWorkpieceWeight-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWorkpieceWeight-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetWorkpieceWeight-response> is deprecated: use dsr_msgs-srv:GetWorkpieceWeight-response instead.")))

(cl:ensure-generic-function 'weight-val :lambda-list '(m))
(cl:defmethod weight-val ((m <GetWorkpieceWeight-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:weight-val is deprecated.  Use dsr_msgs-srv:weight instead.")
  (weight m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetWorkpieceWeight-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWorkpieceWeight-response>) ostream)
  "Serializes a message object of type '<GetWorkpieceWeight-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'weight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWorkpieceWeight-response>) istream)
  "Deserializes a message object of type '<GetWorkpieceWeight-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'weight) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWorkpieceWeight-response>)))
  "Returns string type for a service object of type '<GetWorkpieceWeight-response>"
  "dsr_msgs/GetWorkpieceWeightResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWorkpieceWeight-response)))
  "Returns string type for a service object of type 'GetWorkpieceWeight-response"
  "dsr_msgs/GetWorkpieceWeightResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWorkpieceWeight-response>)))
  "Returns md5sum for a message object of type '<GetWorkpieceWeight-response>"
  "79fbe0854a3099a7d5008b26f92ad15a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWorkpieceWeight-response)))
  "Returns md5sum for a message object of type 'GetWorkpieceWeight-response"
  "79fbe0854a3099a7d5008b26f92ad15a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWorkpieceWeight-response>)))
  "Returns full string definition for message of type '<GetWorkpieceWeight-response>"
  (cl:format cl:nil "float32       weight               # Measured weight, Negative value if error~%bool          success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWorkpieceWeight-response)))
  "Returns full string definition for message of type 'GetWorkpieceWeight-response"
  (cl:format cl:nil "float32       weight               # Measured weight, Negative value if error~%bool          success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWorkpieceWeight-response>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWorkpieceWeight-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWorkpieceWeight-response
    (cl:cons ':weight (weight msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetWorkpieceWeight)))
  'GetWorkpieceWeight-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetWorkpieceWeight)))
  'GetWorkpieceWeight-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWorkpieceWeight)))
  "Returns string type for a service object of type '<GetWorkpieceWeight>"
  "dsr_msgs/GetWorkpieceWeight")