; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude ChangeOperationSpeed-request.msg.html

(cl:defclass <ChangeOperationSpeed-request> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ChangeOperationSpeed-request (<ChangeOperationSpeed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangeOperationSpeed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangeOperationSpeed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ChangeOperationSpeed-request> is deprecated: use dsr_msgs-srv:ChangeOperationSpeed-request instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <ChangeOperationSpeed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:speed-val is deprecated.  Use dsr_msgs-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangeOperationSpeed-request>) ostream)
  "Serializes a message object of type '<ChangeOperationSpeed-request>"
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangeOperationSpeed-request>) istream)
  "Deserializes a message object of type '<ChangeOperationSpeed-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangeOperationSpeed-request>)))
  "Returns string type for a service object of type '<ChangeOperationSpeed-request>"
  "dsr_msgs/ChangeOperationSpeedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeOperationSpeed-request)))
  "Returns string type for a service object of type 'ChangeOperationSpeed-request"
  "dsr_msgs/ChangeOperationSpeedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangeOperationSpeed-request>)))
  "Returns md5sum for a message object of type '<ChangeOperationSpeed-request>"
  "a79c96510dc9b2ea7b678458340a61bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangeOperationSpeed-request)))
  "Returns md5sum for a message object of type 'ChangeOperationSpeed-request"
  "a79c96510dc9b2ea7b678458340a61bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangeOperationSpeed-request>)))
  "Returns full string definition for message of type '<ChangeOperationSpeed-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# change_operation_speed~%#____________________________________________________________________________________________~%~%int8 speed              # operation speed: (1~~100)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangeOperationSpeed-request)))
  "Returns full string definition for message of type 'ChangeOperationSpeed-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# change_operation_speed~%#____________________________________________________________________________________________~%~%int8 speed              # operation speed: (1~~100)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangeOperationSpeed-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangeOperationSpeed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangeOperationSpeed-request
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude ChangeOperationSpeed-response.msg.html

(cl:defclass <ChangeOperationSpeed-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ChangeOperationSpeed-response (<ChangeOperationSpeed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangeOperationSpeed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangeOperationSpeed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<ChangeOperationSpeed-response> is deprecated: use dsr_msgs-srv:ChangeOperationSpeed-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ChangeOperationSpeed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangeOperationSpeed-response>) ostream)
  "Serializes a message object of type '<ChangeOperationSpeed-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangeOperationSpeed-response>) istream)
  "Deserializes a message object of type '<ChangeOperationSpeed-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangeOperationSpeed-response>)))
  "Returns string type for a service object of type '<ChangeOperationSpeed-response>"
  "dsr_msgs/ChangeOperationSpeedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeOperationSpeed-response)))
  "Returns string type for a service object of type 'ChangeOperationSpeed-response"
  "dsr_msgs/ChangeOperationSpeedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangeOperationSpeed-response>)))
  "Returns md5sum for a message object of type '<ChangeOperationSpeed-response>"
  "a79c96510dc9b2ea7b678458340a61bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangeOperationSpeed-response)))
  "Returns md5sum for a message object of type 'ChangeOperationSpeed-response"
  "a79c96510dc9b2ea7b678458340a61bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangeOperationSpeed-response>)))
  "Returns full string definition for message of type '<ChangeOperationSpeed-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangeOperationSpeed-response)))
  "Returns full string definition for message of type 'ChangeOperationSpeed-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangeOperationSpeed-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangeOperationSpeed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangeOperationSpeed-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ChangeOperationSpeed)))
  'ChangeOperationSpeed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ChangeOperationSpeed)))
  'ChangeOperationSpeed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeOperationSpeed)))
  "Returns string type for a service object of type '<ChangeOperationSpeed>"
  "dsr_msgs/ChangeOperationSpeed")