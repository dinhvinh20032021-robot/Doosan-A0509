; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-msg)


;//! \htmlinclude JogMultiAxis.msg.html

(cl:defclass <JogMultiAxis> (roslisp-msg-protocol:ros-message)
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

(cl:defclass JogMultiAxis (<JogMultiAxis>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JogMultiAxis>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JogMultiAxis)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-msg:<JogMultiAxis> is deprecated: use dsr_msgs-msg:JogMultiAxis instead.")))

(cl:ensure-generic-function 'jog_axis-val :lambda-list '(m))
(cl:defmethod jog_axis-val ((m <JogMultiAxis>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:jog_axis-val is deprecated.  Use dsr_msgs-msg:jog_axis instead.")
  (jog_axis m))

(cl:ensure-generic-function 'move_reference-val :lambda-list '(m))
(cl:defmethod move_reference-val ((m <JogMultiAxis>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:move_reference-val is deprecated.  Use dsr_msgs-msg:move_reference instead.")
  (move_reference m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <JogMultiAxis>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:speed-val is deprecated.  Use dsr_msgs-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JogMultiAxis>) ostream)
  "Serializes a message object of type '<JogMultiAxis>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JogMultiAxis>) istream)
  "Deserializes a message object of type '<JogMultiAxis>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JogMultiAxis>)))
  "Returns string type for a message object of type '<JogMultiAxis>"
  "dsr_msgs/JogMultiAxis")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JogMultiAxis)))
  "Returns string type for a message object of type 'JogMultiAxis"
  "dsr_msgs/JogMultiAxis")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JogMultiAxis>)))
  "Returns md5sum for a message object of type '<JogMultiAxis>"
  "e36e615ff2ffad91791b721818ab6be9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JogMultiAxis)))
  "Returns md5sum for a message object of type 'JogMultiAxis"
  "e36e615ff2ffad91791b721818ab6be9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JogMultiAxis>)))
  "Returns full string definition for message of type '<JogMultiAxis>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# multi jog~%# multi jog speed = (250mm/s x 1.73) x unit vecter x speed [%] ~%#____________________________________________________________________________________________~%~%float64[6]  jog_axis          # unit vecter of Task space [Tx, Ty, Tz, Rx, Ry, Rz] : -1.0 ~~ +1.0 ~%int8        move_reference    # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL, 2 : MOVE_REFERENCE_WORLD~%float64     speed             # jog speed [%]~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JogMultiAxis)))
  "Returns full string definition for message of type 'JogMultiAxis"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# multi jog~%# multi jog speed = (250mm/s x 1.73) x unit vecter x speed [%] ~%#____________________________________________________________________________________________~%~%float64[6]  jog_axis          # unit vecter of Task space [Tx, Ty, Tz, Rx, Ry, Rz] : -1.0 ~~ +1.0 ~%int8        move_reference    # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL, 2 : MOVE_REFERENCE_WORLD~%float64     speed             # jog speed [%]~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JogMultiAxis>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'jog_axis) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JogMultiAxis>))
  "Converts a ROS message object to a list"
  (cl:list 'JogMultiAxis
    (cl:cons ':jog_axis (jog_axis msg))
    (cl:cons ':move_reference (move_reference msg))
    (cl:cons ':speed (speed msg))
))
