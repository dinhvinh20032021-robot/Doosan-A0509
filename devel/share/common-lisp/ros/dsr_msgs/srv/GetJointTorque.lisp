; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetJointTorque-request.msg.html

(cl:defclass <GetJointTorque-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetJointTorque-request (<GetJointTorque-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetJointTorque-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetJointTorque-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetJointTorque-request> is deprecated: use dsr_msgs-srv:GetJointTorque-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetJointTorque-request>) ostream)
  "Serializes a message object of type '<GetJointTorque-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetJointTorque-request>) istream)
  "Deserializes a message object of type '<GetJointTorque-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetJointTorque-request>)))
  "Returns string type for a service object of type '<GetJointTorque-request>"
  "dsr_msgs/GetJointTorqueRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointTorque-request)))
  "Returns string type for a service object of type 'GetJointTorque-request"
  "dsr_msgs/GetJointTorqueRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetJointTorque-request>)))
  "Returns md5sum for a message object of type '<GetJointTorque-request>"
  "d2bba428be693e219072ca1b9c1a000a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetJointTorque-request)))
  "Returns md5sum for a message object of type 'GetJointTorque-request"
  "d2bba428be693e219072ca1b9c1a000a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetJointTorque-request>)))
  "Returns full string definition for message of type '<GetJointTorque-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_joint_torque()~%# returns the sensor torque value of the current joint.~%#____________________________________________________________________________________________~%# This service returns the sensor torque value of the current joint.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetJointTorque-request)))
  "Returns full string definition for message of type 'GetJointTorque-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_joint_torque()~%# returns the sensor torque value of the current joint.~%#____________________________________________________________________________________________~%# This service returns the sensor torque value of the current joint.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetJointTorque-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetJointTorque-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetJointTorque-request
))
;//! \htmlinclude GetJointTorque-response.msg.html

(cl:defclass <GetJointTorque-response> (roslisp-msg-protocol:ros-message)
  ((jts
    :reader jts
    :initarg :jts
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetJointTorque-response (<GetJointTorque-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetJointTorque-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetJointTorque-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetJointTorque-response> is deprecated: use dsr_msgs-srv:GetJointTorque-response instead.")))

(cl:ensure-generic-function 'jts-val :lambda-list '(m))
(cl:defmethod jts-val ((m <GetJointTorque-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:jts-val is deprecated.  Use dsr_msgs-srv:jts instead.")
  (jts m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetJointTorque-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetJointTorque-response>) ostream)
  "Serializes a message object of type '<GetJointTorque-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'jts))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetJointTorque-response>) istream)
  "Deserializes a message object of type '<GetJointTorque-response>"
  (cl:setf (cl:slot-value msg 'jts) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'jts)))
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
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetJointTorque-response>)))
  "Returns string type for a service object of type '<GetJointTorque-response>"
  "dsr_msgs/GetJointTorqueResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointTorque-response)))
  "Returns string type for a service object of type 'GetJointTorque-response"
  "dsr_msgs/GetJointTorqueResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetJointTorque-response>)))
  "Returns md5sum for a message object of type '<GetJointTorque-response>"
  "d2bba428be693e219072ca1b9c1a000a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetJointTorque-response)))
  "Returns md5sum for a message object of type 'GetJointTorque-response"
  "d2bba428be693e219072ca1b9c1a000a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetJointTorque-response>)))
  "Returns full string definition for message of type '<GetJointTorque-response>"
  (cl:format cl:nil "float64[6] jts         # value of JTS(Joint Torque Sensor) ~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetJointTorque-response)))
  "Returns full string definition for message of type 'GetJointTorque-response"
  (cl:format cl:nil "float64[6] jts         # value of JTS(Joint Torque Sensor) ~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetJointTorque-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'jts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetJointTorque-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetJointTorque-response
    (cl:cons ':jts (jts msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetJointTorque)))
  'GetJointTorque-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetJointTorque)))
  'GetJointTorque-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointTorque)))
  "Returns string type for a service object of type '<GetJointTorque>"
  "dsr_msgs/GetJointTorque")