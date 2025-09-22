; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetVelJRT-request.msg.html

(cl:defclass <SetVelJRT-request> (roslisp-msg-protocol:ros-message)
  ((vel
    :reader vel
    :initarg :vel
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SetVelJRT-request (<SetVelJRT-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVelJRT-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVelJRT-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetVelJRT-request> is deprecated: use dsr_msgs-srv:SetVelJRT-request instead.")))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <SetVelJRT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:vel-val is deprecated.  Use dsr_msgs-srv:vel instead.")
  (vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVelJRT-request>) ostream)
  "Serializes a message object of type '<SetVelJRT-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'vel))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVelJRT-request>) istream)
  "Deserializes a message object of type '<SetVelJRT-request>"
  (cl:setf (cl:slot-value msg 'vel) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'vel)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVelJRT-request>)))
  "Returns string type for a service object of type '<SetVelJRT-request>"
  "dsr_msgs/SetVelJRTRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVelJRT-request)))
  "Returns string type for a service object of type 'SetVelJRT-request"
  "dsr_msgs/SetVelJRTRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVelJRT-request>)))
  "Returns md5sum for a message object of type '<SetVelJRT-request>"
  "54c890dbfd325b76bab7dbd45237c7df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVelJRT-request)))
  "Returns md5sum for a message object of type 'SetVelJRT-request"
  "54c890dbfd325b76bab7dbd45237c7df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVelJRT-request>)))
  "Returns full string definition for message of type '<SetVelJRT-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_velj_rt~%#____________________________________________________________________________________________~%float64[6] vel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVelJRT-request)))
  "Returns full string definition for message of type 'SetVelJRT-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_velj_rt~%#____________________________________________________________________________________________~%float64[6] vel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVelJRT-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'vel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVelJRT-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVelJRT-request
    (cl:cons ':vel (vel msg))
))
;//! \htmlinclude SetVelJRT-response.msg.html

(cl:defclass <SetVelJRT-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetVelJRT-response (<SetVelJRT-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVelJRT-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVelJRT-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetVelJRT-response> is deprecated: use dsr_msgs-srv:SetVelJRT-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetVelJRT-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVelJRT-response>) ostream)
  "Serializes a message object of type '<SetVelJRT-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVelJRT-response>) istream)
  "Deserializes a message object of type '<SetVelJRT-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVelJRT-response>)))
  "Returns string type for a service object of type '<SetVelJRT-response>"
  "dsr_msgs/SetVelJRTResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVelJRT-response)))
  "Returns string type for a service object of type 'SetVelJRT-response"
  "dsr_msgs/SetVelJRTResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVelJRT-response>)))
  "Returns md5sum for a message object of type '<SetVelJRT-response>"
  "54c890dbfd325b76bab7dbd45237c7df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVelJRT-response)))
  "Returns md5sum for a message object of type 'SetVelJRT-response"
  "54c890dbfd325b76bab7dbd45237c7df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVelJRT-response>)))
  "Returns full string definition for message of type '<SetVelJRT-response>"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVelJRT-response)))
  "Returns full string definition for message of type 'SetVelJRT-response"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVelJRT-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVelJRT-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVelJRT-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetVelJRT)))
  'SetVelJRT-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetVelJRT)))
  'SetVelJRT-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVelJRT)))
  "Returns string type for a service object of type '<SetVelJRT>"
  "dsr_msgs/SetVelJRT")