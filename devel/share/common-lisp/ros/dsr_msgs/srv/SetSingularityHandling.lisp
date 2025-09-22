; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetSingularityHandling-request.msg.html

(cl:defclass <SetSingularityHandling-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetSingularityHandling-request (<SetSingularityHandling-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSingularityHandling-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSingularityHandling-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetSingularityHandling-request> is deprecated: use dsr_msgs-srv:SetSingularityHandling-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <SetSingularityHandling-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:mode-val is deprecated.  Use dsr_msgs-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSingularityHandling-request>) ostream)
  "Serializes a message object of type '<SetSingularityHandling-request>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSingularityHandling-request>) istream)
  "Deserializes a message object of type '<SetSingularityHandling-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSingularityHandling-request>)))
  "Returns string type for a service object of type '<SetSingularityHandling-request>"
  "dsr_msgs/SetSingularityHandlingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSingularityHandling-request)))
  "Returns string type for a service object of type 'SetSingularityHandling-request"
  "dsr_msgs/SetSingularityHandlingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSingularityHandling-request>)))
  "Returns md5sum for a message object of type '<SetSingularityHandling-request>"
  "5da578ca824a6682cf704b593bb1a59e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSingularityHandling-request)))
  "Returns md5sum for a message object of type 'SetSingularityHandling-request"
  "5da578ca824a6682cf704b593bb1a59e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSingularityHandling-request>)))
  "Returns full string definition for message of type '<SetSingularityHandling-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_singularity_handling~%# ~%#____________________________________________________________________________________________~%~%int8       mode         # DR_AVOID(0)     : Automatic avoidance mode~%                        # DR_TASK_STOP(1) : Deceleration/ Warning/ Task termination~%                        # DR_VAR_VEL(2)   : Variable velocity mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSingularityHandling-request)))
  "Returns full string definition for message of type 'SetSingularityHandling-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_singularity_handling~%# ~%#____________________________________________________________________________________________~%~%int8       mode         # DR_AVOID(0)     : Automatic avoidance mode~%                        # DR_TASK_STOP(1) : Deceleration/ Warning/ Task termination~%                        # DR_VAR_VEL(2)   : Variable velocity mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSingularityHandling-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSingularityHandling-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSingularityHandling-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude SetSingularityHandling-response.msg.html

(cl:defclass <SetSingularityHandling-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetSingularityHandling-response (<SetSingularityHandling-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSingularityHandling-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSingularityHandling-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetSingularityHandling-response> is deprecated: use dsr_msgs-srv:SetSingularityHandling-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetSingularityHandling-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSingularityHandling-response>) ostream)
  "Serializes a message object of type '<SetSingularityHandling-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSingularityHandling-response>) istream)
  "Deserializes a message object of type '<SetSingularityHandling-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSingularityHandling-response>)))
  "Returns string type for a service object of type '<SetSingularityHandling-response>"
  "dsr_msgs/SetSingularityHandlingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSingularityHandling-response)))
  "Returns string type for a service object of type 'SetSingularityHandling-response"
  "dsr_msgs/SetSingularityHandlingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSingularityHandling-response>)))
  "Returns md5sum for a message object of type '<SetSingularityHandling-response>"
  "5da578ca824a6682cf704b593bb1a59e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSingularityHandling-response)))
  "Returns md5sum for a message object of type 'SetSingularityHandling-response"
  "5da578ca824a6682cf704b593bb1a59e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSingularityHandling-response>)))
  "Returns full string definition for message of type '<SetSingularityHandling-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSingularityHandling-response)))
  "Returns full string definition for message of type 'SetSingularityHandling-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSingularityHandling-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSingularityHandling-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSingularityHandling-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetSingularityHandling)))
  'SetSingularityHandling-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetSingularityHandling)))
  'SetSingularityHandling-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSingularityHandling)))
  "Returns string type for a service object of type '<SetSingularityHandling>"
  "dsr_msgs/SetSingularityHandling")