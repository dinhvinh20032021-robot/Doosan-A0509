; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude ResetWorkpieceWeight-request.msg.html

(cl:defclass <ResetWorkpieceWeight-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ResetWorkpieceWeight-request (<ResetWorkpieceWeight-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetWorkpieceWeight-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetWorkpieceWeight-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ResetWorkpieceWeight-request> is deprecated: use dsr_msgs-srv:ResetWorkpieceWeight-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetWorkpieceWeight-request>) ostream)
  "Serializes a message object of type '<ResetWorkpieceWeight-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetWorkpieceWeight-request>) istream)
  "Deserializes a message object of type '<ResetWorkpieceWeight-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetWorkpieceWeight-request>)))
  "Returns string type for a service object of type '<ResetWorkpieceWeight-request>"
  "dsr_msgs/ResetWorkpieceWeightRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetWorkpieceWeight-request)))
  "Returns string type for a service object of type 'ResetWorkpieceWeight-request"
  "dsr_msgs/ResetWorkpieceWeightRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetWorkpieceWeight-request>)))
  "Returns md5sum for a message object of type '<ResetWorkpieceWeight-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetWorkpieceWeight-request)))
  "Returns md5sum for a message object of type 'ResetWorkpieceWeight-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetWorkpieceWeight-request>)))
  "Returns full string definition for message of type '<ResetWorkpieceWeight-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# reset_workpiece_weight~%# Initializes the weight data of the material to initialize the algorithm before measuring the weight of the material.~%# ____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetWorkpieceWeight-request)))
  "Returns full string definition for message of type 'ResetWorkpieceWeight-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# reset_workpiece_weight~%# Initializes the weight data of the material to initialize the algorithm before measuring the weight of the material.~%# ____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetWorkpieceWeight-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetWorkpieceWeight-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetWorkpieceWeight-request
))
;//! \htmlinclude ResetWorkpieceWeight-response.msg.html

(cl:defclass <ResetWorkpieceWeight-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ResetWorkpieceWeight-response (<ResetWorkpieceWeight-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetWorkpieceWeight-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetWorkpieceWeight-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ResetWorkpieceWeight-response> is deprecated: use dsr_msgs-srv:ResetWorkpieceWeight-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ResetWorkpieceWeight-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetWorkpieceWeight-response>) ostream)
  "Serializes a message object of type '<ResetWorkpieceWeight-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetWorkpieceWeight-response>) istream)
  "Deserializes a message object of type '<ResetWorkpieceWeight-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetWorkpieceWeight-response>)))
  "Returns string type for a service object of type '<ResetWorkpieceWeight-response>"
  "dsr_msgs/ResetWorkpieceWeightResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetWorkpieceWeight-response)))
  "Returns string type for a service object of type 'ResetWorkpieceWeight-response"
  "dsr_msgs/ResetWorkpieceWeightResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetWorkpieceWeight-response>)))
  "Returns md5sum for a message object of type '<ResetWorkpieceWeight-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetWorkpieceWeight-response)))
  "Returns md5sum for a message object of type 'ResetWorkpieceWeight-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetWorkpieceWeight-response>)))
  "Returns full string definition for message of type '<ResetWorkpieceWeight-response>"
  (cl:format cl:nil "bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetWorkpieceWeight-response)))
  "Returns full string definition for message of type 'ResetWorkpieceWeight-response"
  (cl:format cl:nil "bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetWorkpieceWeight-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetWorkpieceWeight-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetWorkpieceWeight-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetWorkpieceWeight)))
  'ResetWorkpieceWeight-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetWorkpieceWeight)))
  'ResetWorkpieceWeight-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetWorkpieceWeight)))
  "Returns string type for a service object of type '<ResetWorkpieceWeight>"
  "dsr_msgs/ResetWorkpieceWeight")