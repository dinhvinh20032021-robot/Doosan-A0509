; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-msg)


;//! \htmlinclude TorqueRTStream.msg.html

(cl:defclass <TorqueRTStream> (roslisp-msg-protocol:ros-message)
  ((tor
    :reader tor
    :initarg :tor
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0))
)

(cl:defclass TorqueRTStream (<TorqueRTStream>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TorqueRTStream>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TorqueRTStream)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-msg:<TorqueRTStream> is deprecated: use dsr_msgs-msg:TorqueRTStream instead.")))

(cl:ensure-generic-function 'tor-val :lambda-list '(m))
(cl:defmethod tor-val ((m <TorqueRTStream>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:tor-val is deprecated.  Use dsr_msgs-msg:tor instead.")
  (tor m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <TorqueRTStream>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:time-val is deprecated.  Use dsr_msgs-msg:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TorqueRTStream>) ostream)
  "Serializes a message object of type '<TorqueRTStream>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'tor))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TorqueRTStream>) istream)
  "Deserializes a message object of type '<TorqueRTStream>"
  (cl:setf (cl:slot-value msg 'tor) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'tor)))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TorqueRTStream>)))
  "Returns string type for a message object of type '<TorqueRTStream>"
  "dsr_msgs/TorqueRTStream")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TorqueRTStream)))
  "Returns string type for a message object of type 'TorqueRTStream"
  "dsr_msgs/TorqueRTStream")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TorqueRTStream>)))
  "Returns md5sum for a message object of type '<TorqueRTStream>"
  "f87591a62093bf8e8f3ad8edb630f87d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TorqueRTStream)))
  "Returns md5sum for a message object of type 'TorqueRTStream"
  "f87591a62093bf8e8f3ad8edb630f87d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TorqueRTStream>)))
  "Returns full string definition for message of type '<TorqueRTStream>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# torque_rt~%# ~%#____________________________________________________________________________________________~%~%float64[6] tor               # motor torque~%float64    time              # time~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TorqueRTStream)))
  "Returns full string definition for message of type 'TorqueRTStream"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# torque_rt~%# ~%#____________________________________________________________________________________________~%~%float64[6] tor               # motor torque~%float64    time              # time~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TorqueRTStream>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'tor) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TorqueRTStream>))
  "Converts a ROS message object to a list"
  (cl:list 'TorqueRTStream
    (cl:cons ':tor (tor msg))
    (cl:cons ':time (time msg))
))
