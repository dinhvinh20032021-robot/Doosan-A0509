; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude WriteDataRT-request.msg.html

(cl:defclass <WriteDataRT-request> (roslisp-msg-protocol:ros-message)
  ((external_force_torque
    :reader external_force_torque
    :initarg :external_force_torque
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (external_digital_input
    :reader external_digital_input
    :initarg :external_digital_input
    :type cl:integer
    :initform 0)
   (external_digital_output
    :reader external_digital_output
    :initarg :external_digital_output
    :type cl:integer
    :initform 0)
   (external_analog_input
    :reader external_analog_input
    :initarg :external_analog_input
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (external_analog_output
    :reader external_analog_output
    :initarg :external_analog_output
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass WriteDataRT-request (<WriteDataRT-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WriteDataRT-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WriteDataRT-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<WriteDataRT-request> is deprecated: use dsr_msgs-srv:WriteDataRT-request instead.")))

(cl:ensure-generic-function 'external_force_torque-val :lambda-list '(m))
(cl:defmethod external_force_torque-val ((m <WriteDataRT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:external_force_torque-val is deprecated.  Use dsr_msgs-srv:external_force_torque instead.")
  (external_force_torque m))

(cl:ensure-generic-function 'external_digital_input-val :lambda-list '(m))
(cl:defmethod external_digital_input-val ((m <WriteDataRT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:external_digital_input-val is deprecated.  Use dsr_msgs-srv:external_digital_input instead.")
  (external_digital_input m))

(cl:ensure-generic-function 'external_digital_output-val :lambda-list '(m))
(cl:defmethod external_digital_output-val ((m <WriteDataRT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:external_digital_output-val is deprecated.  Use dsr_msgs-srv:external_digital_output instead.")
  (external_digital_output m))

(cl:ensure-generic-function 'external_analog_input-val :lambda-list '(m))
(cl:defmethod external_analog_input-val ((m <WriteDataRT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:external_analog_input-val is deprecated.  Use dsr_msgs-srv:external_analog_input instead.")
  (external_analog_input m))

(cl:ensure-generic-function 'external_analog_output-val :lambda-list '(m))
(cl:defmethod external_analog_output-val ((m <WriteDataRT-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:external_analog_output-val is deprecated.  Use dsr_msgs-srv:external_analog_output instead.")
  (external_analog_output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WriteDataRT-request>) ostream)
  "Serializes a message object of type '<WriteDataRT-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'external_force_torque))
  (cl:let* ((signed (cl:slot-value msg 'external_digital_input)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'external_digital_output)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'external_analog_input))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'external_analog_output))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WriteDataRT-request>) istream)
  "Deserializes a message object of type '<WriteDataRT-request>"
  (cl:setf (cl:slot-value msg 'external_force_torque) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'external_force_torque)))
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'external_digital_input) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'external_digital_output) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:setf (cl:slot-value msg 'external_analog_input) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'external_analog_input)))
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
  (cl:setf (cl:slot-value msg 'external_analog_output) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'external_analog_output)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WriteDataRT-request>)))
  "Returns string type for a service object of type '<WriteDataRT-request>"
  "dsr_msgs/WriteDataRTRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WriteDataRT-request)))
  "Returns string type for a service object of type 'WriteDataRT-request"
  "dsr_msgs/WriteDataRTRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WriteDataRT-request>)))
  "Returns md5sum for a message object of type '<WriteDataRT-request>"
  "57fbcf1b749a9e589d5aa77f87216c0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WriteDataRT-request)))
  "Returns md5sum for a message object of type 'WriteDataRT-request"
  "57fbcf1b749a9e589d5aa77f87216c0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WriteDataRT-request>)))
  "Returns full string definition for message of type '<WriteDataRT-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# write_data_rt~%#____________________________________________________________________________________________~%float64[6] external_force_torque~%int32      external_digital_input~%int32      external_digital_output~%float64[6] external_analog_input~%float64[6] external_analog_output~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WriteDataRT-request)))
  "Returns full string definition for message of type 'WriteDataRT-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# write_data_rt~%#____________________________________________________________________________________________~%float64[6] external_force_torque~%int32      external_digital_input~%int32      external_digital_output~%float64[6] external_analog_input~%float64[6] external_analog_output~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WriteDataRT-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'external_force_torque) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'external_analog_input) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'external_analog_output) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WriteDataRT-request>))
  "Converts a ROS message object to a list"
  (cl:list 'WriteDataRT-request
    (cl:cons ':external_force_torque (external_force_torque msg))
    (cl:cons ':external_digital_input (external_digital_input msg))
    (cl:cons ':external_digital_output (external_digital_output msg))
    (cl:cons ':external_analog_input (external_analog_input msg))
    (cl:cons ':external_analog_output (external_analog_output msg))
))
;//! \htmlinclude WriteDataRT-response.msg.html

(cl:defclass <WriteDataRT-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass WriteDataRT-response (<WriteDataRT-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WriteDataRT-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WriteDataRT-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<WriteDataRT-response> is deprecated: use dsr_msgs-srv:WriteDataRT-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <WriteDataRT-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WriteDataRT-response>) ostream)
  "Serializes a message object of type '<WriteDataRT-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WriteDataRT-response>) istream)
  "Deserializes a message object of type '<WriteDataRT-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WriteDataRT-response>)))
  "Returns string type for a service object of type '<WriteDataRT-response>"
  "dsr_msgs/WriteDataRTResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WriteDataRT-response)))
  "Returns string type for a service object of type 'WriteDataRT-response"
  "dsr_msgs/WriteDataRTResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WriteDataRT-response>)))
  "Returns md5sum for a message object of type '<WriteDataRT-response>"
  "57fbcf1b749a9e589d5aa77f87216c0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WriteDataRT-response)))
  "Returns md5sum for a message object of type 'WriteDataRT-response"
  "57fbcf1b749a9e589d5aa77f87216c0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WriteDataRT-response>)))
  "Returns full string definition for message of type '<WriteDataRT-response>"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WriteDataRT-response)))
  "Returns full string definition for message of type 'WriteDataRT-response"
  (cl:format cl:nil "bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WriteDataRT-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WriteDataRT-response>))
  "Converts a ROS message object to a list"
  (cl:list 'WriteDataRT-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'WriteDataRT)))
  'WriteDataRT-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'WriteDataRT)))
  'WriteDataRT-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WriteDataRT)))
  "Returns string type for a service object of type '<WriteDataRT>"
  "dsr_msgs/WriteDataRT")