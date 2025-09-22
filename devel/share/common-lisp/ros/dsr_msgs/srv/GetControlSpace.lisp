; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetControlSpace-request.msg.html

(cl:defclass <GetControlSpace-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetControlSpace-request (<GetControlSpace-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControlSpace-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControlSpace-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetControlSpace-request> is deprecated: use dsr_msgs-srv:GetControlSpace-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControlSpace-request>) ostream)
  "Serializes a message object of type '<GetControlSpace-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControlSpace-request>) istream)
  "Deserializes a message object of type '<GetControlSpace-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControlSpace-request>)))
  "Returns string type for a service object of type '<GetControlSpace-request>"
  "dsr_msgs/GetControlSpaceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControlSpace-request)))
  "Returns string type for a service object of type 'GetControlSpace-request"
  "dsr_msgs/GetControlSpaceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControlSpace-request>)))
  "Returns md5sum for a message object of type '<GetControlSpace-request>"
  "1a5780af2202e44900c27ba310bfe10c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControlSpace-request)))
  "Returns md5sum for a message object of type 'GetControlSpace-request"
  "1a5780af2202e44900c27ba310bfe10c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControlSpace-request>)))
  "Returns full string definition for message of type '<GetControlSpace-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_control_space()  ~%#____________________________________________________________________________________________~%# This service returns the current control space.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControlSpace-request)))
  "Returns full string definition for message of type 'GetControlSpace-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_control_space()  ~%#____________________________________________________________________________________________~%# This service returns the current control space.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControlSpace-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControlSpace-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControlSpace-request
))
;//! \htmlinclude GetControlSpace-response.msg.html

(cl:defclass <GetControlSpace-response> (roslisp-msg-protocol:ros-message)
  ((space
    :reader space
    :initarg :space
    :type cl:fixnum
    :initform 0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetControlSpace-response (<GetControlSpace-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetControlSpace-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetControlSpace-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetControlSpace-response> is deprecated: use dsr_msgs-srv:GetControlSpace-response instead.")))

(cl:ensure-generic-function 'space-val :lambda-list '(m))
(cl:defmethod space-val ((m <GetControlSpace-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:space-val is deprecated.  Use dsr_msgs-srv:space instead.")
  (space m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetControlSpace-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetControlSpace-response>) ostream)
  "Serializes a message object of type '<GetControlSpace-response>"
  (cl:let* ((signed (cl:slot-value msg 'space)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetControlSpace-response>) istream)
  "Deserializes a message object of type '<GetControlSpace-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'space) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetControlSpace-response>)))
  "Returns string type for a service object of type '<GetControlSpace-response>"
  "dsr_msgs/GetControlSpaceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControlSpace-response)))
  "Returns string type for a service object of type 'GetControlSpace-response"
  "dsr_msgs/GetControlSpaceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetControlSpace-response>)))
  "Returns md5sum for a message object of type '<GetControlSpace-response>"
  "1a5780af2202e44900c27ba310bfe10c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetControlSpace-response)))
  "Returns md5sum for a message object of type 'GetControlSpace-response"
  "1a5780af2202e44900c27ba310bfe10c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetControlSpace-response>)))
  "Returns full string definition for message of type '<GetControlSpace-response>"
  (cl:format cl:nil "int8    space        # Control mode : Joint space control(1), Task space control(2)~%bool    success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetControlSpace-response)))
  "Returns full string definition for message of type 'GetControlSpace-response"
  (cl:format cl:nil "int8    space        # Control mode : Joint space control(1), Task space control(2)~%bool    success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetControlSpace-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetControlSpace-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetControlSpace-response
    (cl:cons ':space (space msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetControlSpace)))
  'GetControlSpace-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetControlSpace)))
  'GetControlSpace-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetControlSpace)))
  "Returns string type for a service object of type '<GetControlSpace>"
  "dsr_msgs/GetControlSpace")