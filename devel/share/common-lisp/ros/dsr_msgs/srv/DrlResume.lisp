; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude DrlResume-request.msg.html

(cl:defclass <DrlResume-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DrlResume-request (<DrlResume-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DrlResume-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DrlResume-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<DrlResume-request> is deprecated: use dsr_msgs-srv:DrlResume-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DrlResume-request>) ostream)
  "Serializes a message object of type '<DrlResume-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DrlResume-request>) istream)
  "Deserializes a message object of type '<DrlResume-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DrlResume-request>)))
  "Returns string type for a service object of type '<DrlResume-request>"
  "dsr_msgs/DrlResumeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrlResume-request)))
  "Returns string type for a service object of type 'DrlResume-request"
  "dsr_msgs/DrlResumeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DrlResume-request>)))
  "Returns md5sum for a message object of type '<DrlResume-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DrlResume-request)))
  "Returns md5sum for a message object of type 'DrlResume-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DrlResume-request>)))
  "Returns full string definition for message of type '<DrlResume-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# drl_script_resume  ~%# It is a service to resume the currently paused DRL program in the robot controller.    ~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DrlResume-request)))
  "Returns full string definition for message of type 'DrlResume-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# drl_script_resume  ~%# It is a service to resume the currently paused DRL program in the robot controller.    ~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DrlResume-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DrlResume-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DrlResume-request
))
;//! \htmlinclude DrlResume-response.msg.html

(cl:defclass <DrlResume-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DrlResume-response (<DrlResume-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DrlResume-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DrlResume-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<DrlResume-response> is deprecated: use dsr_msgs-srv:DrlResume-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DrlResume-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DrlResume-response>) ostream)
  "Serializes a message object of type '<DrlResume-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DrlResume-response>) istream)
  "Deserializes a message object of type '<DrlResume-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DrlResume-response>)))
  "Returns string type for a service object of type '<DrlResume-response>"
  "dsr_msgs/DrlResumeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrlResume-response)))
  "Returns string type for a service object of type 'DrlResume-response"
  "dsr_msgs/DrlResumeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DrlResume-response>)))
  "Returns md5sum for a message object of type '<DrlResume-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DrlResume-response)))
  "Returns md5sum for a message object of type 'DrlResume-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DrlResume-response>)))
  "Returns full string definition for message of type '<DrlResume-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DrlResume-response)))
  "Returns full string definition for message of type 'DrlResume-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DrlResume-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DrlResume-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DrlResume-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DrlResume)))
  'DrlResume-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DrlResume)))
  'DrlResume-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DrlResume)))
  "Returns string type for a service object of type '<DrlResume>"
  "dsr_msgs/DrlResume")