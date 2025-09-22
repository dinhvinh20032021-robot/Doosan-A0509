; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude MovePeriodic-request.msg.html

(cl:defclass <MovePeriodic-request> (roslisp-msg-protocol:ros-message)
  ((amp
    :reader amp
    :initarg :amp
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (periodic
    :reader periodic
    :initarg :periodic
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (acc
    :reader acc
    :initarg :acc
    :type cl:float
    :initform 0.0)
   (repeat
    :reader repeat
    :initarg :repeat
    :type cl:fixnum
    :initform 0)
   (ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0)
   (syncType
    :reader syncType
    :initarg :syncType
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MovePeriodic-request (<MovePeriodic-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MovePeriodic-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MovePeriodic-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<MovePeriodic-request> is deprecated: use dsr_msgs-srv:MovePeriodic-request instead.")))

(cl:ensure-generic-function 'amp-val :lambda-list '(m))
(cl:defmethod amp-val ((m <MovePeriodic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:amp-val is deprecated.  Use dsr_msgs-srv:amp instead.")
  (amp m))

(cl:ensure-generic-function 'periodic-val :lambda-list '(m))
(cl:defmethod periodic-val ((m <MovePeriodic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:periodic-val is deprecated.  Use dsr_msgs-srv:periodic instead.")
  (periodic m))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <MovePeriodic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:acc-val is deprecated.  Use dsr_msgs-srv:acc instead.")
  (acc m))

(cl:ensure-generic-function 'repeat-val :lambda-list '(m))
(cl:defmethod repeat-val ((m <MovePeriodic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:repeat-val is deprecated.  Use dsr_msgs-srv:repeat instead.")
  (repeat m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <MovePeriodic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))

(cl:ensure-generic-function 'syncType-val :lambda-list '(m))
(cl:defmethod syncType-val ((m <MovePeriodic-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:syncType-val is deprecated.  Use dsr_msgs-srv:syncType instead.")
  (syncType m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MovePeriodic-request>) ostream)
  "Serializes a message object of type '<MovePeriodic-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'amp))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'periodic))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'acc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'repeat)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'syncType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MovePeriodic-request>) istream)
  "Deserializes a message object of type '<MovePeriodic-request>"
  (cl:setf (cl:slot-value msg 'amp) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'amp)))
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
  (cl:setf (cl:slot-value msg 'periodic) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'periodic)))
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
    (cl:setf (cl:slot-value msg 'acc) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'repeat) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'syncType) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MovePeriodic-request>)))
  "Returns string type for a service object of type '<MovePeriodic-request>"
  "dsr_msgs/MovePeriodicRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MovePeriodic-request)))
  "Returns string type for a service object of type 'MovePeriodic-request"
  "dsr_msgs/MovePeriodicRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MovePeriodic-request>)))
  "Returns md5sum for a message object of type '<MovePeriodic-request>"
  "d18cadb0c21c585bdf0c4a1e6878f1a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MovePeriodic-request)))
  "Returns md5sum for a message object of type 'MovePeriodic-request"
  "d18cadb0c21c585bdf0c4a1e6878f1a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MovePeriodic-request>)))
  "Returns full string definition for message of type '<MovePeriodic-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# move_periodic  ~%#____________________________________________________________________________________________~%~%float64[6] amp              # Amplitude (motion between -amp and +amp) [mm] or [deg]   ~%float64[6] periodic         # Period (time for 1 cycle) [sec]~%float64    acc              # Acc-, dec- time [sec] ~%int8       repeat           # Repetition count ~%int8       ref  #= 1        # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)~%                            # <DR_WORLD is only available in M2.40 or later ~%~%int8       syncType #=0     # SYNC = 0, ASYNC = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MovePeriodic-request)))
  "Returns full string definition for message of type 'MovePeriodic-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# move_periodic  ~%#____________________________________________________________________________________________~%~%float64[6] amp              # Amplitude (motion between -amp and +amp) [mm] or [deg]   ~%float64[6] periodic         # Period (time for 1 cycle) [sec]~%float64    acc              # Acc-, dec- time [sec] ~%int8       repeat           # Repetition count ~%int8       ref  #= 1        # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)~%                            # <DR_WORLD is only available in M2.40 or later ~%~%int8       syncType #=0     # SYNC = 0, ASYNC = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MovePeriodic-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'amp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'periodic) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MovePeriodic-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MovePeriodic-request
    (cl:cons ':amp (amp msg))
    (cl:cons ':periodic (periodic msg))
    (cl:cons ':acc (acc msg))
    (cl:cons ':repeat (repeat msg))
    (cl:cons ':ref (ref msg))
    (cl:cons ':syncType (syncType msg))
))
;//! \htmlinclude MovePeriodic-response.msg.html

(cl:defclass <MovePeriodic-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MovePeriodic-response (<MovePeriodic-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MovePeriodic-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MovePeriodic-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<MovePeriodic-response> is deprecated: use dsr_msgs-srv:MovePeriodic-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <MovePeriodic-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MovePeriodic-response>) ostream)
  "Serializes a message object of type '<MovePeriodic-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MovePeriodic-response>) istream)
  "Deserializes a message object of type '<MovePeriodic-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MovePeriodic-response>)))
  "Returns string type for a service object of type '<MovePeriodic-response>"
  "dsr_msgs/MovePeriodicResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MovePeriodic-response)))
  "Returns string type for a service object of type 'MovePeriodic-response"
  "dsr_msgs/MovePeriodicResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MovePeriodic-response>)))
  "Returns md5sum for a message object of type '<MovePeriodic-response>"
  "d18cadb0c21c585bdf0c4a1e6878f1a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MovePeriodic-response)))
  "Returns md5sum for a message object of type 'MovePeriodic-response"
  "d18cadb0c21c585bdf0c4a1e6878f1a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MovePeriodic-response>)))
  "Returns full string definition for message of type '<MovePeriodic-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MovePeriodic-response)))
  "Returns full string definition for message of type 'MovePeriodic-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MovePeriodic-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MovePeriodic-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MovePeriodic-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MovePeriodic)))
  'MovePeriodic-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MovePeriodic)))
  'MovePeriodic-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MovePeriodic)))
  "Returns string type for a service object of type '<MovePeriodic>"
  "dsr_msgs/MovePeriodic")