; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetSolutionSpace-request.msg.html

(cl:defclass <GetSolutionSpace-request> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GetSolutionSpace-request (<GetSolutionSpace-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSolutionSpace-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSolutionSpace-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetSolutionSpace-request> is deprecated: use dsr_msgs-srv:GetSolutionSpace-request instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <GetSolutionSpace-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:pos-val is deprecated.  Use dsr_msgs-srv:pos instead.")
  (pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSolutionSpace-request>) ostream)
  "Serializes a message object of type '<GetSolutionSpace-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'pos))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSolutionSpace-request>) istream)
  "Deserializes a message object of type '<GetSolutionSpace-request>"
  (cl:setf (cl:slot-value msg 'pos) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'pos)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSolutionSpace-request>)))
  "Returns string type for a service object of type '<GetSolutionSpace-request>"
  "dsr_msgs/GetSolutionSpaceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSolutionSpace-request)))
  "Returns string type for a service object of type 'GetSolutionSpace-request"
  "dsr_msgs/GetSolutionSpaceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSolutionSpace-request>)))
  "Returns md5sum for a message object of type '<GetSolutionSpace-request>"
  "27abbe19a7d41c61a3226076e3320ebf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSolutionSpace-request)))
  "Returns md5sum for a message object of type 'GetSolutionSpace-request"
  "27abbe19a7d41c61a3226076e3320ebf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSolutionSpace-request>)))
  "Returns full string definition for message of type '<GetSolutionSpace-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_solution_space(pos)  ~%#____________________________________________________________________________________________~%# This service obtains the solution space value.~%~%float64[6] pos               # joint angle list [degree] ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSolutionSpace-request)))
  "Returns full string definition for message of type 'GetSolutionSpace-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_solution_space(pos)  ~%#____________________________________________________________________________________________~%# This service obtains the solution space value.~%~%float64[6] pos               # joint angle list [degree] ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSolutionSpace-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSolutionSpace-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSolutionSpace-request
    (cl:cons ':pos (pos msg))
))
;//! \htmlinclude GetSolutionSpace-response.msg.html

(cl:defclass <GetSolutionSpace-response> (roslisp-msg-protocol:ros-message)
  ((sol_space
    :reader sol_space
    :initarg :sol_space
    :type cl:fixnum
    :initform 0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetSolutionSpace-response (<GetSolutionSpace-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSolutionSpace-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSolutionSpace-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetSolutionSpace-response> is deprecated: use dsr_msgs-srv:GetSolutionSpace-response instead.")))

(cl:ensure-generic-function 'sol_space-val :lambda-list '(m))
(cl:defmethod sol_space-val ((m <GetSolutionSpace-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:sol_space-val is deprecated.  Use dsr_msgs-srv:sol_space instead.")
  (sol_space m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetSolutionSpace-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSolutionSpace-response>) ostream)
  "Serializes a message object of type '<GetSolutionSpace-response>"
  (cl:let* ((signed (cl:slot-value msg 'sol_space)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSolutionSpace-response>) istream)
  "Deserializes a message object of type '<GetSolutionSpace-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sol_space) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSolutionSpace-response>)))
  "Returns string type for a service object of type '<GetSolutionSpace-response>"
  "dsr_msgs/GetSolutionSpaceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSolutionSpace-response)))
  "Returns string type for a service object of type 'GetSolutionSpace-response"
  "dsr_msgs/GetSolutionSpaceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSolutionSpace-response>)))
  "Returns md5sum for a message object of type '<GetSolutionSpace-response>"
  "27abbe19a7d41c61a3226076e3320ebf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSolutionSpace-response)))
  "Returns md5sum for a message object of type 'GetSolutionSpace-response"
  "27abbe19a7d41c61a3226076e3320ebf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSolutionSpace-response>)))
  "Returns full string definition for message of type '<GetSolutionSpace-response>"
  (cl:format cl:nil "int8       sol_space         # solution space : 0 ~~ 7~%bool       success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSolutionSpace-response)))
  "Returns full string definition for message of type 'GetSolutionSpace-response"
  (cl:format cl:nil "int8       sol_space         # solution space : 0 ~~ 7~%bool       success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSolutionSpace-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSolutionSpace-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSolutionSpace-response
    (cl:cons ':sol_space (sol_space msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSolutionSpace)))
  'GetSolutionSpace-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSolutionSpace)))
  'GetSolutionSpace-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSolutionSpace)))
  "Returns string type for a service object of type '<GetSolutionSpace>"
  "dsr_msgs/GetSolutionSpace")