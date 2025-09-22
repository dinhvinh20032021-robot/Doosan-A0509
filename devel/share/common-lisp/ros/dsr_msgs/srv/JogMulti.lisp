; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude JogMulti-request.msg.html

(cl:defclass <JogMulti-request> (roslisp-msg-protocol:ros-message)
  ((jog_axis
    :reader jog_axis
    :initarg :jog_axis
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (move_reference
    :reader move_reference
    :initarg :move_reference
    :type cl:fixnum
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass JogMulti-request (<JogMulti-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JogMulti-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JogMulti-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<JogMulti-request> is deprecated: use dsr_msgs-srv:JogMulti-request instead.")))

(cl:ensure-generic-function 'jog_axis-val :lambda-list '(m))
(cl:defmethod jog_axis-val ((m <JogMulti-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:jog_axis-val is deprecated.  Use dsr_msgs-srv:jog_axis instead.")
  (jog_axis m))

(cl:ensure-generic-function 'move_reference-val :lambda-list '(m))
(cl:defmethod move_reference-val ((m <JogMulti-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:move_reference-val is deprecated.  Use dsr_msgs-srv:move_reference instead.")
  (move_reference m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <JogMulti-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:speed-val is deprecated.  Use dsr_msgs-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JogMulti-request>) ostream)
  "Serializes a message object of type '<JogMulti-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'jog_axis))
  (cl:let* ((signed (cl:slot-value msg 'move_reference)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JogMulti-request>) istream)
  "Deserializes a message object of type '<JogMulti-request>"
  (cl:setf (cl:slot-value msg 'jog_axis) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'jog_axis)))
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
      (cl:setf (cl:slot-value msg 'move_reference) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JogMulti-request>)))
  "Returns string type for a service object of type '<JogMulti-request>"
  "dsr_msgs/JogMultiRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JogMulti-request)))
  "Returns string type for a service object of type 'JogMulti-request"
  "dsr_msgs/JogMultiRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JogMulti-request>)))
  "Returns md5sum for a message object of type '<JogMulti-request>"
  "fc4c9a5dbff2b2bad258eb5f8e8ba682")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JogMulti-request)))
  "Returns md5sum for a message object of type 'JogMulti-request"
  "fc4c9a5dbff2b2bad258eb5f8e8ba682")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JogMulti-request>)))
  "Returns full string definition for message of type '<JogMulti-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# multi jog speed = (250mm/s x 1.73) x unit vecter x speed [%] ~%#____________________________________________________________________________________________~%~%float64[6] jog_axis    # unit vecter of Task space [Tx, Ty, Tz, Rx, Ry, Rz] : -1.0 ~~ +1.0 ~%int8 move_reference    # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL, 2 : MOVE_REFERENCE_WORLD~%float64 speed          # jog speed [%]  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JogMulti-request)))
  "Returns full string definition for message of type 'JogMulti-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# multi jog speed = (250mm/s x 1.73) x unit vecter x speed [%] ~%#____________________________________________________________________________________________~%~%float64[6] jog_axis    # unit vecter of Task space [Tx, Ty, Tz, Rx, Ry, Rz] : -1.0 ~~ +1.0 ~%int8 move_reference    # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL, 2 : MOVE_REFERENCE_WORLD~%float64 speed          # jog speed [%]  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JogMulti-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'jog_axis) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JogMulti-request>))
  "Converts a ROS message object to a list"
  (cl:list 'JogMulti-request
    (cl:cons ':jog_axis (jog_axis msg))
    (cl:cons ':move_reference (move_reference msg))
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude JogMulti-response.msg.html

(cl:defclass <JogMulti-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass JogMulti-response (<JogMulti-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JogMulti-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JogMulti-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<JogMulti-response> is deprecated: use dsr_msgs-srv:JogMulti-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <JogMulti-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JogMulti-response>) ostream)
  "Serializes a message object of type '<JogMulti-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JogMulti-response>) istream)
  "Deserializes a message object of type '<JogMulti-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JogMulti-response>)))
  "Returns string type for a service object of type '<JogMulti-response>"
  "dsr_msgs/JogMultiResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JogMulti-response)))
  "Returns string type for a service object of type 'JogMulti-response"
  "dsr_msgs/JogMultiResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JogMulti-response>)))
  "Returns md5sum for a message object of type '<JogMulti-response>"
  "fc4c9a5dbff2b2bad258eb5f8e8ba682")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JogMulti-response)))
  "Returns md5sum for a message object of type 'JogMulti-response"
  "fc4c9a5dbff2b2bad258eb5f8e8ba682")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JogMulti-response>)))
  "Returns full string definition for message of type '<JogMulti-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JogMulti-response)))
  "Returns full string definition for message of type 'JogMulti-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JogMulti-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JogMulti-response>))
  "Converts a ROS message object to a list"
  (cl:list 'JogMulti-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'JogMulti)))
  'JogMulti-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'JogMulti)))
  'JogMulti-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JogMulti)))
  "Returns string type for a service object of type '<JogMulti>"
  "dsr_msgs/JogMulti")