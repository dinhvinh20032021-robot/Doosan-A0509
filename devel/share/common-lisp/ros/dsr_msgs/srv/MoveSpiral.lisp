; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude MoveSpiral-request.msg.html

(cl:defclass <MoveSpiral-request> (roslisp-msg-protocol:ros-message)
  ((revolution
    :reader revolution
    :initarg :revolution
    :type cl:float
    :initform 0.0)
   (maxRadius
    :reader maxRadius
    :initarg :maxRadius
    :type cl:float
    :initform 0.0)
   (maxLength
    :reader maxLength
    :initarg :maxLength
    :type cl:float
    :initform 0.0)
   (vel
    :reader vel
    :initarg :vel
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (acc
    :reader acc
    :initarg :acc
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0)
   (taskAxis
    :reader taskAxis
    :initarg :taskAxis
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

(cl:defclass MoveSpiral-request (<MoveSpiral-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveSpiral-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveSpiral-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<MoveSpiral-request> is deprecated: use dsr_msgs-srv:MoveSpiral-request instead.")))

(cl:ensure-generic-function 'revolution-val :lambda-list '(m))
(cl:defmethod revolution-val ((m <MoveSpiral-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:revolution-val is deprecated.  Use dsr_msgs-srv:revolution instead.")
  (revolution m))

(cl:ensure-generic-function 'maxRadius-val :lambda-list '(m))
(cl:defmethod maxRadius-val ((m <MoveSpiral-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:maxRadius-val is deprecated.  Use dsr_msgs-srv:maxRadius instead.")
  (maxRadius m))

(cl:ensure-generic-function 'maxLength-val :lambda-list '(m))
(cl:defmethod maxLength-val ((m <MoveSpiral-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:maxLength-val is deprecated.  Use dsr_msgs-srv:maxLength instead.")
  (maxLength m))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <MoveSpiral-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:vel-val is deprecated.  Use dsr_msgs-srv:vel instead.")
  (vel m))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <MoveSpiral-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:acc-val is deprecated.  Use dsr_msgs-srv:acc instead.")
  (acc m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <MoveSpiral-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:time-val is deprecated.  Use dsr_msgs-srv:time instead.")
  (time m))

(cl:ensure-generic-function 'taskAxis-val :lambda-list '(m))
(cl:defmethod taskAxis-val ((m <MoveSpiral-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:taskAxis-val is deprecated.  Use dsr_msgs-srv:taskAxis instead.")
  (taskAxis m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <MoveSpiral-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))

(cl:ensure-generic-function 'syncType-val :lambda-list '(m))
(cl:defmethod syncType-val ((m <MoveSpiral-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:syncType-val is deprecated.  Use dsr_msgs-srv:syncType instead.")
  (syncType m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveSpiral-request>) ostream)
  "Serializes a message object of type '<MoveSpiral-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'revolution))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'maxRadius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'maxLength))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
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
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'taskAxis)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'syncType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveSpiral-request>) istream)
  "Deserializes a message object of type '<MoveSpiral-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'revolution) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxRadius) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxLength) (roslisp-utils:decode-double-float-bits bits)))
  (cl:setf (cl:slot-value msg 'vel) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'vel)))
    (cl:dotimes (i 2)
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
  (cl:setf (cl:slot-value msg 'acc) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'acc)))
    (cl:dotimes (i 2)
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'taskAxis) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'syncType) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveSpiral-request>)))
  "Returns string type for a service object of type '<MoveSpiral-request>"
  "dsr_msgs/MoveSpiralRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveSpiral-request)))
  "Returns string type for a service object of type 'MoveSpiral-request"
  "dsr_msgs/MoveSpiralRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveSpiral-request>)))
  "Returns md5sum for a message object of type '<MoveSpiral-request>"
  "fc2c602cd43258b5eb06926d790138ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveSpiral-request)))
  "Returns md5sum for a message object of type 'MoveSpiral-request"
  "fc2c602cd43258b5eb06926d790138ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveSpiral-request>)))
  "Returns full string definition for message of type '<MoveSpiral-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# move_sprial  ~%#____________________________________________________________________________________________~%~%float64    revolution       # Total number of revolutions ~%float64    maxRadius        # Final spiral radius [mm]~%float64    maxLength        # Distance moved in the axis direction [mm]~%float64[2] vel              # set velocity: [mm/sec], [deg/sec]~%float64[2] acc              # set acceleration: [mm/sec2], [deg/sec2]~%float64    time #= 0.0      # Total execution time <sec> ~%int8       taskAxis         # TASK_AXIS_X = 0, TASK_AXIS_Y = 1, TASK_AXIS_Z = 2   ~%int8       ref  #= 1        # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)~%                            # <DR_WORLD is only available in M2.40 or later ~%int8       syncType #=0     # SYNC = 0, ASYNC = 1 ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveSpiral-request)))
  "Returns full string definition for message of type 'MoveSpiral-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# move_sprial  ~%#____________________________________________________________________________________________~%~%float64    revolution       # Total number of revolutions ~%float64    maxRadius        # Final spiral radius [mm]~%float64    maxLength        # Distance moved in the axis direction [mm]~%float64[2] vel              # set velocity: [mm/sec], [deg/sec]~%float64[2] acc              # set acceleration: [mm/sec2], [deg/sec2]~%float64    time #= 0.0      # Total execution time <sec> ~%int8       taskAxis         # TASK_AXIS_X = 0, TASK_AXIS_Y = 1, TASK_AXIS_Z = 2   ~%int8       ref  #= 1        # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)~%                            # <DR_WORLD is only available in M2.40 or later ~%int8       syncType #=0     # SYNC = 0, ASYNC = 1 ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveSpiral-request>))
  (cl:+ 0
     8
     8
     8
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'vel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'acc) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveSpiral-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveSpiral-request
    (cl:cons ':revolution (revolution msg))
    (cl:cons ':maxRadius (maxRadius msg))
    (cl:cons ':maxLength (maxLength msg))
    (cl:cons ':vel (vel msg))
    (cl:cons ':acc (acc msg))
    (cl:cons ':time (time msg))
    (cl:cons ':taskAxis (taskAxis msg))
    (cl:cons ':ref (ref msg))
    (cl:cons ':syncType (syncType msg))
))
;//! \htmlinclude MoveSpiral-response.msg.html

(cl:defclass <MoveSpiral-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoveSpiral-response (<MoveSpiral-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveSpiral-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveSpiral-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<MoveSpiral-response> is deprecated: use dsr_msgs-srv:MoveSpiral-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <MoveSpiral-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveSpiral-response>) ostream)
  "Serializes a message object of type '<MoveSpiral-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveSpiral-response>) istream)
  "Deserializes a message object of type '<MoveSpiral-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveSpiral-response>)))
  "Returns string type for a service object of type '<MoveSpiral-response>"
  "dsr_msgs/MoveSpiralResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveSpiral-response)))
  "Returns string type for a service object of type 'MoveSpiral-response"
  "dsr_msgs/MoveSpiralResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveSpiral-response>)))
  "Returns md5sum for a message object of type '<MoveSpiral-response>"
  "fc2c602cd43258b5eb06926d790138ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveSpiral-response)))
  "Returns md5sum for a message object of type 'MoveSpiral-response"
  "fc2c602cd43258b5eb06926d790138ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveSpiral-response>)))
  "Returns full string definition for message of type '<MoveSpiral-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveSpiral-response)))
  "Returns full string definition for message of type 'MoveSpiral-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveSpiral-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveSpiral-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveSpiral-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveSpiral)))
  'MoveSpiral-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveSpiral)))
  'MoveSpiral-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveSpiral)))
  "Returns string type for a service object of type '<MoveSpiral>"
  "dsr_msgs/MoveSpiral")