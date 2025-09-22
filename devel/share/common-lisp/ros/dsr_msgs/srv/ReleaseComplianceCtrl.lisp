; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude ReleaseComplianceCtrl-request.msg.html

(cl:defclass <ReleaseComplianceCtrl-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ReleaseComplianceCtrl-request (<ReleaseComplianceCtrl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReleaseComplianceCtrl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReleaseComplianceCtrl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ReleaseComplianceCtrl-request> is deprecated: use dsr_msgs-srv:ReleaseComplianceCtrl-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReleaseComplianceCtrl-request>) ostream)
  "Serializes a message object of type '<ReleaseComplianceCtrl-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReleaseComplianceCtrl-request>) istream)
  "Deserializes a message object of type '<ReleaseComplianceCtrl-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReleaseComplianceCtrl-request>)))
  "Returns string type for a service object of type '<ReleaseComplianceCtrl-request>"
  "dsr_msgs/ReleaseComplianceCtrlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReleaseComplianceCtrl-request)))
  "Returns string type for a service object of type 'ReleaseComplianceCtrl-request"
  "dsr_msgs/ReleaseComplianceCtrlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReleaseComplianceCtrl-request>)))
  "Returns md5sum for a message object of type '<ReleaseComplianceCtrl-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReleaseComplianceCtrl-request)))
  "Returns md5sum for a message object of type 'ReleaseComplianceCtrl-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReleaseComplianceCtrl-request>)))
  "Returns full string definition for message of type '<ReleaseComplianceCtrl-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# release_compliance_ctrl  ~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReleaseComplianceCtrl-request)))
  "Returns full string definition for message of type 'ReleaseComplianceCtrl-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# release_compliance_ctrl  ~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReleaseComplianceCtrl-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReleaseComplianceCtrl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReleaseComplianceCtrl-request
))
;//! \htmlinclude ReleaseComplianceCtrl-response.msg.html

(cl:defclass <ReleaseComplianceCtrl-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ReleaseComplianceCtrl-response (<ReleaseComplianceCtrl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReleaseComplianceCtrl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReleaseComplianceCtrl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ReleaseComplianceCtrl-response> is deprecated: use dsr_msgs-srv:ReleaseComplianceCtrl-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ReleaseComplianceCtrl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReleaseComplianceCtrl-response>) ostream)
  "Serializes a message object of type '<ReleaseComplianceCtrl-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReleaseComplianceCtrl-response>) istream)
  "Deserializes a message object of type '<ReleaseComplianceCtrl-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReleaseComplianceCtrl-response>)))
  "Returns string type for a service object of type '<ReleaseComplianceCtrl-response>"
  "dsr_msgs/ReleaseComplianceCtrlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReleaseComplianceCtrl-response)))
  "Returns string type for a service object of type 'ReleaseComplianceCtrl-response"
  "dsr_msgs/ReleaseComplianceCtrlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReleaseComplianceCtrl-response>)))
  "Returns md5sum for a message object of type '<ReleaseComplianceCtrl-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReleaseComplianceCtrl-response)))
  "Returns md5sum for a message object of type 'ReleaseComplianceCtrl-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReleaseComplianceCtrl-response>)))
  "Returns full string definition for message of type '<ReleaseComplianceCtrl-response>"
  (cl:format cl:nil "bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReleaseComplianceCtrl-response)))
  "Returns full string definition for message of type 'ReleaseComplianceCtrl-response"
  (cl:format cl:nil "bool       success ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReleaseComplianceCtrl-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReleaseComplianceCtrl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReleaseComplianceCtrl-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReleaseComplianceCtrl)))
  'ReleaseComplianceCtrl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReleaseComplianceCtrl)))
  'ReleaseComplianceCtrl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReleaseComplianceCtrl)))
  "Returns string type for a service object of type '<ReleaseComplianceCtrl>"
  "dsr_msgs/ReleaseComplianceCtrl")