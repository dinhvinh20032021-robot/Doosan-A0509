; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetCurrentSolutionSpace-request.msg.html

(cl:defclass <GetCurrentSolutionSpace-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetCurrentSolutionSpace-request (<GetCurrentSolutionSpace-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentSolutionSpace-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentSolutionSpace-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCurrentSolutionSpace-request> is deprecated: use dsr_msgs-srv:GetCurrentSolutionSpace-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentSolutionSpace-request>) ostream)
  "Serializes a message object of type '<GetCurrentSolutionSpace-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentSolutionSpace-request>) istream)
  "Deserializes a message object of type '<GetCurrentSolutionSpace-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentSolutionSpace-request>)))
  "Returns string type for a service object of type '<GetCurrentSolutionSpace-request>"
  "dsr_msgs/GetCurrentSolutionSpaceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentSolutionSpace-request)))
  "Returns string type for a service object of type 'GetCurrentSolutionSpace-request"
  "dsr_msgs/GetCurrentSolutionSpaceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentSolutionSpace-request>)))
  "Returns md5sum for a message object of type '<GetCurrentSolutionSpace-request>"
  "bacee50609041091b8cb888ffb4bd562")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentSolutionSpace-request)))
  "Returns md5sum for a message object of type 'GetCurrentSolutionSpace-request"
  "bacee50609041091b8cb888ffb4bd562")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentSolutionSpace-request>)))
  "Returns full string definition for message of type '<GetCurrentSolutionSpace-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_current_solution_space~%#____________________________________________________________________________________________~%# This service returns the current solution space value.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentSolutionSpace-request)))
  "Returns full string definition for message of type 'GetCurrentSolutionSpace-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_current_solution_space~%#____________________________________________________________________________________________~%# This service returns the current solution space value.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentSolutionSpace-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentSolutionSpace-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentSolutionSpace-request
))
;//! \htmlinclude GetCurrentSolutionSpace-response.msg.html

(cl:defclass <GetCurrentSolutionSpace-response> (roslisp-msg-protocol:ros-message)
  ((sol_space
    :reader sol_space
    :initarg :sol_space
    :type cl:fixnum
    :initform 0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetCurrentSolutionSpace-response (<GetCurrentSolutionSpace-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentSolutionSpace-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentSolutionSpace-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetCurrentSolutionSpace-response> is deprecated: use dsr_msgs-srv:GetCurrentSolutionSpace-response instead.")))

(cl:ensure-generic-function 'sol_space-val :lambda-list '(m))
(cl:defmethod sol_space-val ((m <GetCurrentSolutionSpace-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:sol_space-val is deprecated.  Use dsr_msgs-srv:sol_space instead.")
  (sol_space m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetCurrentSolutionSpace-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentSolutionSpace-response>) ostream)
  "Serializes a message object of type '<GetCurrentSolutionSpace-response>"
  (cl:let* ((signed (cl:slot-value msg 'sol_space)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentSolutionSpace-response>) istream)
  "Deserializes a message object of type '<GetCurrentSolutionSpace-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sol_space) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentSolutionSpace-response>)))
  "Returns string type for a service object of type '<GetCurrentSolutionSpace-response>"
  "dsr_msgs/GetCurrentSolutionSpaceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentSolutionSpace-response)))
  "Returns string type for a service object of type 'GetCurrentSolutionSpace-response"
  "dsr_msgs/GetCurrentSolutionSpaceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentSolutionSpace-response>)))
  "Returns md5sum for a message object of type '<GetCurrentSolutionSpace-response>"
  "bacee50609041091b8cb888ffb4bd562")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentSolutionSpace-response)))
  "Returns md5sum for a message object of type 'GetCurrentSolutionSpace-response"
  "bacee50609041091b8cb888ffb4bd562")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentSolutionSpace-response>)))
  "Returns full string definition for message of type '<GetCurrentSolutionSpace-response>"
  (cl:format cl:nil "int8        sol_space         # solution space : 0 ~~ 7~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentSolutionSpace-response)))
  "Returns full string definition for message of type 'GetCurrentSolutionSpace-response"
  (cl:format cl:nil "int8        sol_space         # solution space : 0 ~~ 7~%bool        success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentSolutionSpace-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentSolutionSpace-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentSolutionSpace-response
    (cl:cons ':sol_space (sol_space msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCurrentSolutionSpace)))
  'GetCurrentSolutionSpace-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCurrentSolutionSpace)))
  'GetCurrentSolutionSpace-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentSolutionSpace)))
  "Returns string type for a service object of type '<GetCurrentSolutionSpace>"
  "dsr_msgs/GetCurrentSolutionSpace")