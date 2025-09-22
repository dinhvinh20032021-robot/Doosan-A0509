; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetAccJRT-request.msg.html

(cl:defclass <SetAccJRT-request> (roslisp-msg-protocol:ros-message)
  ((acc
    :reader acc
    :initarg :acc
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SetAccJRT-request (<SetAccJRT-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAccJRT-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAccJRT-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetAccJRT-request> is deprecated: use dsr_msgs-srv:SetAccJRT-request instead.")))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <SetAccJRT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:acc-val is deprecated.  Use dsr_msgs-srv:acc instead.")
  (acc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAccJRT-request>) ostream)
  "Serializes a message object of type '<SetAccJRT-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'acc))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAccJRT-request>) istream)
  "Deserializes a message object of type '<SetAccJRT-request>"
  (cl:setf (cl:slot-value msg 'acc) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'acc)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAccJRT-request>)))
  "Returns string type for a service object of type '<SetAccJRT-request>"
  "dsr_msgs/SetAccJRTRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAccJRT-request)))
  "Returns string type for a service object of type 'SetAccJRT-request"
  "dsr_msgs/SetAccJRTRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAccJRT-request>)))
  "Returns md5sum for a message object of type '<SetAccJRT-request>"
  "813f535e4d5aa4d44e3a153e7c2700e3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAccJRT-request)))
  "Returns md5sum for a message object of type 'SetAccJRT-request"
  "813f535e4d5aa4d44e3a153e7c2700e3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAccJRT-request>)))
  "Returns full string definition for message of type '<SetAccJRT-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_accj_rt~%#____________________________________________________________________________________________~%float64[6] acc~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAccJRT-request)))
  "Returns full string definition for message of type 'SetAccJRT-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_accj_rt~%#____________________________________________________________________________________________~%float64[6] acc~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAccJRT-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'acc) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAccJRT-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAccJRT-request
    (cl:cons ':acc (acc msg))
))
;//! \htmlinclude SetAccJRT-response.msg.html

(cl:defclass <SetAccJRT-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetAccJRT-response (<SetAccJRT-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAccJRT-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAccJRT-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetAccJRT-response> is deprecated: use dsr_msgs-srv:SetAccJRT-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetAccJRT-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAccJRT-response>) ostream)
  "Serializes a message object of type '<SetAccJRT-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAccJRT-response>) istream)
  "Deserializes a message object of type '<SetAccJRT-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAccJRT-response>)))
  "Returns string type for a service object of type '<SetAccJRT-response>"
  "dsr_msgs/SetAccJRTResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAccJRT-response)))
  "Returns string type for a service object of type 'SetAccJRT-response"
  "dsr_msgs/SetAccJRTResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAccJRT-response>)))
  "Returns md5sum for a message object of type '<SetAccJRT-response>"
  "813f535e4d5aa4d44e3a153e7c2700e3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAccJRT-response)))
  "Returns md5sum for a message object of type 'SetAccJRT-response"
  "813f535e4d5aa4d44e3a153e7c2700e3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAccJRT-response>)))
  "Returns full string definition for message of type '<SetAccJRT-response>"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAccJRT-response)))
  "Returns full string definition for message of type 'SetAccJRT-response"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAccJRT-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAccJRT-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAccJRT-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetAccJRT)))
  'SetAccJRT-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetAccJRT)))
  'SetAccJRT-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAccJRT)))
  "Returns string type for a service object of type '<SetAccJRT>"
  "dsr_msgs/SetAccJRT")