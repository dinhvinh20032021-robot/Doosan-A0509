; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetUserCartCoord-request.msg.html

(cl:defclass <GetUserCartCoord-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetUserCartCoord-request (<GetUserCartCoord-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetUserCartCoord-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetUserCartCoord-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetUserCartCoord-request> is deprecated: use dsr_msgs-srv:GetUserCartCoord-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <GetUserCartCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:id-val is deprecated.  Use dsr_msgs-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetUserCartCoord-request>) ostream)
  "Serializes a message object of type '<GetUserCartCoord-request>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetUserCartCoord-request>) istream)
  "Deserializes a message object of type '<GetUserCartCoord-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetUserCartCoord-request>)))
  "Returns string type for a service object of type '<GetUserCartCoord-request>"
  "dsr_msgs/GetUserCartCoordRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetUserCartCoord-request)))
  "Returns string type for a service object of type 'GetUserCartCoord-request"
  "dsr_msgs/GetUserCartCoordRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetUserCartCoord-request>)))
  "Returns md5sum for a message object of type '<GetUserCartCoord-request>"
  "a933e25ab5287ae9b6e7ebd1be20a647")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetUserCartCoord-request)))
  "Returns md5sum for a message object of type 'GetUserCartCoord-request"
  "a933e25ab5287ae9b6e7ebd1be20a647")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetUserCartCoord-request>)))
  "Returns full string definition for message of type '<GetUserCartCoord-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# posx, ref = get_user_cart_coord(id)   ~%#____________________________________________________________________________________________~%# This service is only available in M2.50 or later~%~%int8       id                # ID of user coord ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetUserCartCoord-request)))
  "Returns full string definition for message of type 'GetUserCartCoord-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# posx, ref = get_user_cart_coord(id)   ~%#____________________________________________________________________________________________~%# This service is only available in M2.50 or later~%~%int8       id                # ID of user coord ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetUserCartCoord-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetUserCartCoord-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetUserCartCoord-request
    (cl:cons ':id (id msg))
))
;//! \htmlinclude GetUserCartCoord-response.msg.html

(cl:defclass <GetUserCartCoord-response> (roslisp-msg-protocol:ros-message)
  ((conv_posx
    :reader conv_posx
    :initarg :conv_posx
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (ref
    :reader ref
    :initarg :ref
    :type cl:fixnum
    :initform 0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetUserCartCoord-response (<GetUserCartCoord-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetUserCartCoord-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetUserCartCoord-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetUserCartCoord-response> is deprecated: use dsr_msgs-srv:GetUserCartCoord-response instead.")))

(cl:ensure-generic-function 'conv_posx-val :lambda-list '(m))
(cl:defmethod conv_posx-val ((m <GetUserCartCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:conv_posx-val is deprecated.  Use dsr_msgs-srv:conv_posx instead.")
  (conv_posx m))

(cl:ensure-generic-function 'ref-val :lambda-list '(m))
(cl:defmethod ref-val ((m <GetUserCartCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:ref-val is deprecated.  Use dsr_msgs-srv:ref instead.")
  (ref m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetUserCartCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetUserCartCoord-response>) ostream)
  "Serializes a message object of type '<GetUserCartCoord-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'conv_posx))
  (cl:let* ((signed (cl:slot-value msg 'ref)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetUserCartCoord-response>) istream)
  "Deserializes a message object of type '<GetUserCartCoord-response>"
  (cl:setf (cl:slot-value msg 'conv_posx) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'conv_posx)))
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
      (cl:setf (cl:slot-value msg 'ref) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetUserCartCoord-response>)))
  "Returns string type for a service object of type '<GetUserCartCoord-response>"
  "dsr_msgs/GetUserCartCoordResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetUserCartCoord-response)))
  "Returns string type for a service object of type 'GetUserCartCoord-response"
  "dsr_msgs/GetUserCartCoordResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetUserCartCoord-response>)))
  "Returns md5sum for a message object of type '<GetUserCartCoord-response>"
  "a933e25ab5287ae9b6e7ebd1be20a647")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetUserCartCoord-response)))
  "Returns md5sum for a message object of type 'GetUserCartCoord-response"
  "a933e25ab5287ae9b6e7ebd1be20a647")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetUserCartCoord-response>)))
  "Returns full string definition for message of type '<GetUserCartCoord-response>"
  (cl:format cl:nil "float64[6] conv_posx         # task pos(posx)  ~%int8       ref               # Reference coordinate of the coordinate to get~%bool       success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetUserCartCoord-response)))
  "Returns full string definition for message of type 'GetUserCartCoord-response"
  (cl:format cl:nil "float64[6] conv_posx         # task pos(posx)  ~%int8       ref               # Reference coordinate of the coordinate to get~%bool       success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetUserCartCoord-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'conv_posx) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetUserCartCoord-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetUserCartCoord-response
    (cl:cons ':conv_posx (conv_posx msg))
    (cl:cons ':ref (ref msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetUserCartCoord)))
  'GetUserCartCoord-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetUserCartCoord)))
  'GetUserCartCoord-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetUserCartCoord)))
  "Returns string type for a service object of type '<GetUserCartCoord>"
  "dsr_msgs/GetUserCartCoord")