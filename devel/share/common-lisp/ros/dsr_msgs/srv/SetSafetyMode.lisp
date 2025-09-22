; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetSafetyMode-request.msg.html

(cl:defclass <SetSafetyMode-request> (roslisp-msg-protocol:ros-message)
  ((safety_mode
    :reader safety_mode
    :initarg :safety_mode
    :type cl:fixnum
    :initform 0)
   (safety_mode_event
    :reader safety_mode_event
    :initarg :safety_mode_event
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetSafetyMode-request (<SetSafetyMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSafetyMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSafetyMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetSafetyMode-request> is deprecated: use dsr_msgs-srv:SetSafetyMode-request instead.")))

(cl:ensure-generic-function 'safety_mode-val :lambda-list '(m))
(cl:defmethod safety_mode-val ((m <SetSafetyMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:safety_mode-val is deprecated.  Use dsr_msgs-srv:safety_mode instead.")
  (safety_mode m))

(cl:ensure-generic-function 'safety_mode_event-val :lambda-list '(m))
(cl:defmethod safety_mode_event-val ((m <SetSafetyMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:safety_mode_event-val is deprecated.  Use dsr_msgs-srv:safety_mode_event instead.")
  (safety_mode_event m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSafetyMode-request>) ostream)
  "Serializes a message object of type '<SetSafetyMode-request>"
  (cl:let* ((signed (cl:slot-value msg 'safety_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'safety_mode_event)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSafetyMode-request>) istream)
  "Deserializes a message object of type '<SetSafetyMode-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'safety_mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'safety_mode_event) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSafetyMode-request>)))
  "Returns string type for a service object of type '<SetSafetyMode-request>"
  "dsr_msgs/SetSafetyModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafetyMode-request)))
  "Returns string type for a service object of type 'SetSafetyMode-request"
  "dsr_msgs/SetSafetyModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSafetyMode-request>)))
  "Returns md5sum for a message object of type '<SetSafetyMode-request>"
  "82c2323d83064354d4a93feb6b800608")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSafetyMode-request)))
  "Returns md5sum for a message object of type 'SetSafetyMode-request"
  "82c2323d83064354d4a93feb6b800608")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSafetyMode-request>)))
  "Returns full string definition for message of type '<SetSafetyMode-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_safety_mode~%#____________________________________________________________________________________________~%~%int8 safety_mode  ~%int8 safety_mode_event~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSafetyMode-request)))
  "Returns full string definition for message of type 'SetSafetyMode-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_safety_mode~%#____________________________________________________________________________________________~%~%int8 safety_mode  ~%int8 safety_mode_event~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSafetyMode-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSafetyMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSafetyMode-request
    (cl:cons ':safety_mode (safety_mode msg))
    (cl:cons ':safety_mode_event (safety_mode_event msg))
))
;//! \htmlinclude SetSafetyMode-response.msg.html

(cl:defclass <SetSafetyMode-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetSafetyMode-response (<SetSafetyMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSafetyMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSafetyMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetSafetyMode-response> is deprecated: use dsr_msgs-srv:SetSafetyMode-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetSafetyMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSafetyMode-response>) ostream)
  "Serializes a message object of type '<SetSafetyMode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSafetyMode-response>) istream)
  "Deserializes a message object of type '<SetSafetyMode-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSafetyMode-response>)))
  "Returns string type for a service object of type '<SetSafetyMode-response>"
  "dsr_msgs/SetSafetyModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafetyMode-response)))
  "Returns string type for a service object of type 'SetSafetyMode-response"
  "dsr_msgs/SetSafetyModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSafetyMode-response>)))
  "Returns md5sum for a message object of type '<SetSafetyMode-response>"
  "82c2323d83064354d4a93feb6b800608")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSafetyMode-response)))
  "Returns md5sum for a message object of type 'SetSafetyMode-response"
  "82c2323d83064354d4a93feb6b800608")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSafetyMode-response>)))
  "Returns full string definition for message of type '<SetSafetyMode-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSafetyMode-response)))
  "Returns full string definition for message of type 'SetSafetyMode-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSafetyMode-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSafetyMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSafetyMode-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetSafetyMode)))
  'SetSafetyMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetSafetyMode)))
  'SetSafetyMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSafetyMode)))
  "Returns string type for a service object of type '<SetSafetyMode>"
  "dsr_msgs/SetSafetyMode")