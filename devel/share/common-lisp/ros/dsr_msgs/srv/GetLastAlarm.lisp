; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetLastAlarm-request.msg.html

(cl:defclass <GetLastAlarm-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetLastAlarm-request (<GetLastAlarm-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetLastAlarm-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetLastAlarm-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetLastAlarm-request> is deprecated: use dsr_msgs-srv:GetLastAlarm-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetLastAlarm-request>) ostream)
  "Serializes a message object of type '<GetLastAlarm-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetLastAlarm-request>) istream)
  "Deserializes a message object of type '<GetLastAlarm-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetLastAlarm-request>)))
  "Returns string type for a service object of type '<GetLastAlarm-request>"
  "dsr_msgs/GetLastAlarmRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLastAlarm-request)))
  "Returns string type for a service object of type 'GetLastAlarm-request"
  "dsr_msgs/GetLastAlarmRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetLastAlarm-request>)))
  "Returns md5sum for a message object of type '<GetLastAlarm-request>"
  "a6d300dc8ba91bb8020a696fecebd6f2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetLastAlarm-request)))
  "Returns md5sum for a message object of type 'GetLastAlarm-request"
  "a6d300dc8ba91bb8020a696fecebd6f2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetLastAlarm-request>)))
  "Returns full string definition for message of type '<GetLastAlarm-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_last_alarm~%###Struct.LOG_ARARM~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetLastAlarm-request)))
  "Returns full string definition for message of type 'GetLastAlarm-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_last_alarm~%###Struct.LOG_ARARM~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetLastAlarm-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetLastAlarm-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetLastAlarm-request
))
;//! \htmlinclude GetLastAlarm-response.msg.html

(cl:defclass <GetLastAlarm-response> (roslisp-msg-protocol:ros-message)
  ((log_alarm
    :reader log_alarm
    :initarg :log_alarm
    :type dsr_msgs-msg:LogAlarm
    :initform (cl:make-instance 'dsr_msgs-msg:LogAlarm))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetLastAlarm-response (<GetLastAlarm-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetLastAlarm-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetLastAlarm-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetLastAlarm-response> is deprecated: use dsr_msgs-srv:GetLastAlarm-response instead.")))

(cl:ensure-generic-function 'log_alarm-val :lambda-list '(m))
(cl:defmethod log_alarm-val ((m <GetLastAlarm-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:log_alarm-val is deprecated.  Use dsr_msgs-srv:log_alarm instead.")
  (log_alarm m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetLastAlarm-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetLastAlarm-response>) ostream)
  "Serializes a message object of type '<GetLastAlarm-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'log_alarm) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetLastAlarm-response>) istream)
  "Deserializes a message object of type '<GetLastAlarm-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'log_alarm) istream)
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetLastAlarm-response>)))
  "Returns string type for a service object of type '<GetLastAlarm-response>"
  "dsr_msgs/GetLastAlarmResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLastAlarm-response)))
  "Returns string type for a service object of type 'GetLastAlarm-response"
  "dsr_msgs/GetLastAlarmResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetLastAlarm-response>)))
  "Returns md5sum for a message object of type '<GetLastAlarm-response>"
  "a6d300dc8ba91bb8020a696fecebd6f2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetLastAlarm-response)))
  "Returns md5sum for a message object of type 'GetLastAlarm-response"
  "a6d300dc8ba91bb8020a696fecebd6f2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetLastAlarm-response>)))
  "Returns full string definition for message of type '<GetLastAlarm-response>"
  (cl:format cl:nil "LogAlarm log_alarm~%bool        success~%~%================================================================================~%MSG: dsr_msgs/LogAlarm~%#____________________________________________________________________________________________~%# log of alarm~%#____________________________________________________________________________________________~%~%int32         level~%int32         group~%int32         index~%string[3]     param~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetLastAlarm-response)))
  "Returns full string definition for message of type 'GetLastAlarm-response"
  (cl:format cl:nil "LogAlarm log_alarm~%bool        success~%~%================================================================================~%MSG: dsr_msgs/LogAlarm~%#____________________________________________________________________________________________~%# log of alarm~%#____________________________________________________________________________________________~%~%int32         level~%int32         group~%int32         index~%string[3]     param~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetLastAlarm-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'log_alarm))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetLastAlarm-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetLastAlarm-response
    (cl:cons ':log_alarm (log_alarm msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetLastAlarm)))
  'GetLastAlarm-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetLastAlarm)))
  'GetLastAlarm-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLastAlarm)))
  "Returns string type for a service object of type '<GetLastAlarm>"
  "dsr_msgs/GetLastAlarm")