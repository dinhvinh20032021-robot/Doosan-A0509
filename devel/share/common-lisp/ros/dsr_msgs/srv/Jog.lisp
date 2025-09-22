; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude Jog-request.msg.html

(cl:defclass <Jog-request> (roslisp-msg-protocol:ros-message)
  ((jog_axis
    :reader jog_axis
    :initarg :jog_axis
    :type cl:fixnum
    :initform 0)
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

(cl:defclass Jog-request (<Jog-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Jog-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Jog-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<Jog-request> is deprecated: use dsr_msgs-srv:Jog-request instead.")))

(cl:ensure-generic-function 'jog_axis-val :lambda-list '(m))
(cl:defmethod jog_axis-val ((m <Jog-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:jog_axis-val is deprecated.  Use dsr_msgs-srv:jog_axis instead.")
  (jog_axis m))

(cl:ensure-generic-function 'move_reference-val :lambda-list '(m))
(cl:defmethod move_reference-val ((m <Jog-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:move_reference-val is deprecated.  Use dsr_msgs-srv:move_reference instead.")
  (move_reference m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Jog-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:speed-val is deprecated.  Use dsr_msgs-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Jog-request>) ostream)
  "Serializes a message object of type '<Jog-request>"
  (cl:let* ((signed (cl:slot-value msg 'jog_axis)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Jog-request>) istream)
  "Deserializes a message object of type '<Jog-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'jog_axis) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Jog-request>)))
  "Returns string type for a service object of type '<Jog-request>"
  "dsr_msgs/JogRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Jog-request)))
  "Returns string type for a service object of type 'Jog-request"
  "dsr_msgs/JogRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Jog-request>)))
  "Returns md5sum for a message object of type '<Jog-request>"
  "240aba792f78029c4074628f5a2713fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Jog-request)))
  "Returns md5sum for a message object of type 'Jog-request"
  "240aba792f78029c4074628f5a2713fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Jog-request>)))
  "Returns full string definition for message of type '<Jog-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# single jog~%# single jog speed = (250mm/s) x speed [%] ~%#____________________________________________________________________________________________~%~%int8 jog_axis          # 0 ~~ 5 : JOINT 1 ~~ 6 ~%                       # 6 ~~ 11: TASK 1 ~~ 6 (X,Y,Z,rx,ry,rz)~%int8 move_reference    # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL~%float64 speed          # jog speed [%] : + forward , 0=stop, - backward  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Jog-request)))
  "Returns full string definition for message of type 'Jog-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# single jog~%# single jog speed = (250mm/s) x speed [%] ~%#____________________________________________________________________________________________~%~%int8 jog_axis          # 0 ~~ 5 : JOINT 1 ~~ 6 ~%                       # 6 ~~ 11: TASK 1 ~~ 6 (X,Y,Z,rx,ry,rz)~%int8 move_reference    # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL~%float64 speed          # jog speed [%] : + forward , 0=stop, - backward  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Jog-request>))
  (cl:+ 0
     1
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Jog-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Jog-request
    (cl:cons ':jog_axis (jog_axis msg))
    (cl:cons ':move_reference (move_reference msg))
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude Jog-response.msg.html

(cl:defclass <Jog-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Jog-response (<Jog-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Jog-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Jog-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<Jog-response> is deprecated: use dsr_msgs-srv:Jog-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Jog-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Jog-response>) ostream)
  "Serializes a message object of type '<Jog-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Jog-response>) istream)
  "Deserializes a message object of type '<Jog-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Jog-response>)))
  "Returns string type for a service object of type '<Jog-response>"
  "dsr_msgs/JogResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Jog-response)))
  "Returns string type for a service object of type 'Jog-response"
  "dsr_msgs/JogResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Jog-response>)))
  "Returns md5sum for a message object of type '<Jog-response>"
  "240aba792f78029c4074628f5a2713fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Jog-response)))
  "Returns md5sum for a message object of type 'Jog-response"
  "240aba792f78029c4074628f5a2713fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Jog-response>)))
  "Returns full string definition for message of type '<Jog-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Jog-response)))
  "Returns full string definition for message of type 'Jog-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Jog-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Jog-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Jog-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Jog)))
  'Jog-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Jog)))
  'Jog-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Jog)))
  "Returns string type for a service object of type '<Jog>"
  "dsr_msgs/Jog")