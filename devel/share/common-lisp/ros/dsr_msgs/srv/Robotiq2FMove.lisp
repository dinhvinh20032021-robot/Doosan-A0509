; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude Robotiq2FMove-request.msg.html

(cl:defclass <Robotiq2FMove-request> (roslisp-msg-protocol:ros-message)
  ((width
    :reader width
    :initarg :width
    :type cl:float
    :initform 0.0))
)

(cl:defclass Robotiq2FMove-request (<Robotiq2FMove-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Robotiq2FMove-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Robotiq2FMove-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<Robotiq2FMove-request> is deprecated: use dsr_msgs-srv:Robotiq2FMove-request instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <Robotiq2FMove-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:width-val is deprecated.  Use dsr_msgs-srv:width instead.")
  (width m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Robotiq2FMove-request>) ostream)
  "Serializes a message object of type '<Robotiq2FMove-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Robotiq2FMove-request>) istream)
  "Deserializes a message object of type '<Robotiq2FMove-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Robotiq2FMove-request>)))
  "Returns string type for a service object of type '<Robotiq2FMove-request>"
  "dsr_msgs/Robotiq2FMoveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robotiq2FMove-request)))
  "Returns string type for a service object of type 'Robotiq2FMove-request"
  "dsr_msgs/Robotiq2FMoveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Robotiq2FMove-request>)))
  "Returns md5sum for a message object of type '<Robotiq2FMove-request>"
  "274da388c4c2130f9c892586a4676f9b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Robotiq2FMove-request)))
  "Returns md5sum for a message object of type 'Robotiq2FMove-request"
  "274da388c4c2130f9c892586a4676f9b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Robotiq2FMove-request>)))
  "Returns full string definition for message of type '<Robotiq2FMove-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# robotiq_2f_move~%# open or close the virtual robotiq gripper by input value(width)   ~%#____________________________________________________________________________________________~%~%float64 width # 0.0(open) ~~ 0.8(close)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Robotiq2FMove-request)))
  "Returns full string definition for message of type 'Robotiq2FMove-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# robotiq_2f_move~%# open or close the virtual robotiq gripper by input value(width)   ~%#____________________________________________________________________________________________~%~%float64 width # 0.0(open) ~~ 0.8(close)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Robotiq2FMove-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Robotiq2FMove-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Robotiq2FMove-request
    (cl:cons ':width (width msg))
))
;//! \htmlinclude Robotiq2FMove-response.msg.html

(cl:defclass <Robotiq2FMove-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Robotiq2FMove-response (<Robotiq2FMove-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Robotiq2FMove-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Robotiq2FMove-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<Robotiq2FMove-response> is deprecated: use dsr_msgs-srv:Robotiq2FMove-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Robotiq2FMove-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Robotiq2FMove-response>) ostream)
  "Serializes a message object of type '<Robotiq2FMove-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Robotiq2FMove-response>) istream)
  "Deserializes a message object of type '<Robotiq2FMove-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Robotiq2FMove-response>)))
  "Returns string type for a service object of type '<Robotiq2FMove-response>"
  "dsr_msgs/Robotiq2FMoveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robotiq2FMove-response)))
  "Returns string type for a service object of type 'Robotiq2FMove-response"
  "dsr_msgs/Robotiq2FMoveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Robotiq2FMove-response>)))
  "Returns md5sum for a message object of type '<Robotiq2FMove-response>"
  "274da388c4c2130f9c892586a4676f9b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Robotiq2FMove-response)))
  "Returns md5sum for a message object of type 'Robotiq2FMove-response"
  "274da388c4c2130f9c892586a4676f9b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Robotiq2FMove-response>)))
  "Returns full string definition for message of type '<Robotiq2FMove-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Robotiq2FMove-response)))
  "Returns full string definition for message of type 'Robotiq2FMove-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Robotiq2FMove-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Robotiq2FMove-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Robotiq2FMove-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Robotiq2FMove)))
  'Robotiq2FMove-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Robotiq2FMove)))
  'Robotiq2FMove-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Robotiq2FMove)))
  "Returns string type for a service object of type '<Robotiq2FMove>"
  "dsr_msgs/Robotiq2FMove")