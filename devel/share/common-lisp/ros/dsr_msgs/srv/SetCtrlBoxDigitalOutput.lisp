; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetCtrlBoxDigitalOutput-request.msg.html

(cl:defclass <SetCtrlBoxDigitalOutput-request> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetCtrlBoxDigitalOutput-request (<SetCtrlBoxDigitalOutput-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCtrlBoxDigitalOutput-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCtrlBoxDigitalOutput-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetCtrlBoxDigitalOutput-request> is deprecated: use dsr_msgs-srv:SetCtrlBoxDigitalOutput-request instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <SetCtrlBoxDigitalOutput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:index-val is deprecated.  Use dsr_msgs-srv:index instead.")
  (index m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SetCtrlBoxDigitalOutput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:value-val is deprecated.  Use dsr_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCtrlBoxDigitalOutput-request>) ostream)
  "Serializes a message object of type '<SetCtrlBoxDigitalOutput-request>"
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCtrlBoxDigitalOutput-request>) istream)
  "Deserializes a message object of type '<SetCtrlBoxDigitalOutput-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCtrlBoxDigitalOutput-request>)))
  "Returns string type for a service object of type '<SetCtrlBoxDigitalOutput-request>"
  "dsr_msgs/SetCtrlBoxDigitalOutputRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCtrlBoxDigitalOutput-request)))
  "Returns string type for a service object of type 'SetCtrlBoxDigitalOutput-request"
  "dsr_msgs/SetCtrlBoxDigitalOutputRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCtrlBoxDigitalOutput-request>)))
  "Returns md5sum for a message object of type '<SetCtrlBoxDigitalOutput-request>"
  "fb5dce2e6988e39a422dd00d9d97c412")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCtrlBoxDigitalOutput-request)))
  "Returns md5sum for a message object of type 'SetCtrlBoxDigitalOutput-request"
  "fb5dce2e6988e39a422dd00d9d97c412")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCtrlBoxDigitalOutput-request>)))
  "Returns full string definition for message of type '<SetCtrlBoxDigitalOutput-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_digital_output  ~%#____________________________________________________________________________________________~%~%int8       index    # ctrlbox digital output port(1 ~~ 16)~%int8       value    # 0 : ON, 1 : OFF~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCtrlBoxDigitalOutput-request)))
  "Returns full string definition for message of type 'SetCtrlBoxDigitalOutput-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_digital_output  ~%#____________________________________________________________________________________________~%~%int8       index    # ctrlbox digital output port(1 ~~ 16)~%int8       value    # 0 : ON, 1 : OFF~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCtrlBoxDigitalOutput-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCtrlBoxDigitalOutput-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCtrlBoxDigitalOutput-request
    (cl:cons ':index (index msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude SetCtrlBoxDigitalOutput-response.msg.html

(cl:defclass <SetCtrlBoxDigitalOutput-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetCtrlBoxDigitalOutput-response (<SetCtrlBoxDigitalOutput-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCtrlBoxDigitalOutput-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCtrlBoxDigitalOutput-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetCtrlBoxDigitalOutput-response> is deprecated: use dsr_msgs-srv:SetCtrlBoxDigitalOutput-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetCtrlBoxDigitalOutput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCtrlBoxDigitalOutput-response>) ostream)
  "Serializes a message object of type '<SetCtrlBoxDigitalOutput-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCtrlBoxDigitalOutput-response>) istream)
  "Deserializes a message object of type '<SetCtrlBoxDigitalOutput-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCtrlBoxDigitalOutput-response>)))
  "Returns string type for a service object of type '<SetCtrlBoxDigitalOutput-response>"
  "dsr_msgs/SetCtrlBoxDigitalOutputResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCtrlBoxDigitalOutput-response)))
  "Returns string type for a service object of type 'SetCtrlBoxDigitalOutput-response"
  "dsr_msgs/SetCtrlBoxDigitalOutputResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCtrlBoxDigitalOutput-response>)))
  "Returns md5sum for a message object of type '<SetCtrlBoxDigitalOutput-response>"
  "fb5dce2e6988e39a422dd00d9d97c412")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCtrlBoxDigitalOutput-response)))
  "Returns md5sum for a message object of type 'SetCtrlBoxDigitalOutput-response"
  "fb5dce2e6988e39a422dd00d9d97c412")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCtrlBoxDigitalOutput-response>)))
  "Returns full string definition for message of type '<SetCtrlBoxDigitalOutput-response>"
  (cl:format cl:nil "bool        success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCtrlBoxDigitalOutput-response)))
  "Returns full string definition for message of type 'SetCtrlBoxDigitalOutput-response"
  (cl:format cl:nil "bool        success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCtrlBoxDigitalOutput-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCtrlBoxDigitalOutput-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCtrlBoxDigitalOutput-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetCtrlBoxDigitalOutput)))
  'SetCtrlBoxDigitalOutput-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetCtrlBoxDigitalOutput)))
  'SetCtrlBoxDigitalOutput-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCtrlBoxDigitalOutput)))
  "Returns string type for a service object of type '<SetCtrlBoxDigitalOutput>"
  "dsr_msgs/SetCtrlBoxDigitalOutput")