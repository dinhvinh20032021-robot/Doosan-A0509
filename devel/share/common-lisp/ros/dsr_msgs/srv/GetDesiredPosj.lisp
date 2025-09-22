; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetDesiredPosj-request.msg.html

(cl:defclass <GetDesiredPosj-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetDesiredPosj-request (<GetDesiredPosj-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDesiredPosj-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDesiredPosj-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetDesiredPosj-request> is deprecated: use dsr_msgs-srv:GetDesiredPosj-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDesiredPosj-request>) ostream)
  "Serializes a message object of type '<GetDesiredPosj-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDesiredPosj-request>) istream)
  "Deserializes a message object of type '<GetDesiredPosj-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDesiredPosj-request>)))
  "Returns string type for a service object of type '<GetDesiredPosj-request>"
  "dsr_msgs/GetDesiredPosjRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDesiredPosj-request)))
  "Returns string type for a service object of type 'GetDesiredPosj-request"
  "dsr_msgs/GetDesiredPosjRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDesiredPosj-request>)))
  "Returns md5sum for a message object of type '<GetDesiredPosj-request>"
  "3d228bb409a5fe0306e199313e3ed277")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDesiredPosj-request)))
  "Returns md5sum for a message object of type 'GetDesiredPosj-request"
  "3d228bb409a5fe0306e199313e3ed277")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDesiredPosj-request>)))
  "Returns full string definition for message of type '<GetDesiredPosj-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_desired_posj()  ~%#____________________________________________________________________________________________~%# This service returns the current target joint angle.~%# It cannot be used in the movel, movec, movesx, moveb, move_spiral, or move_periodic service.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDesiredPosj-request)))
  "Returns full string definition for message of type 'GetDesiredPosj-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_desired_posj()  ~%#____________________________________________________________________________________________~%# This service returns the current target joint angle.~%# It cannot be used in the movel, movec, movesx, moveb, move_spiral, or move_periodic service.~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDesiredPosj-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDesiredPosj-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDesiredPosj-request
))
;//! \htmlinclude GetDesiredPosj-response.msg.html

(cl:defclass <GetDesiredPosj-response> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetDesiredPosj-response (<GetDesiredPosj-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDesiredPosj-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDesiredPosj-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetDesiredPosj-response> is deprecated: use dsr_msgs-srv:GetDesiredPosj-response instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <GetDesiredPosj-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:pos-val is deprecated.  Use dsr_msgs-srv:pos instead.")
  (pos m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetDesiredPosj-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDesiredPosj-response>) ostream)
  "Serializes a message object of type '<GetDesiredPosj-response>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDesiredPosj-response>) istream)
  "Deserializes a message object of type '<GetDesiredPosj-response>"
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
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDesiredPosj-response>)))
  "Returns string type for a service object of type '<GetDesiredPosj-response>"
  "dsr_msgs/GetDesiredPosjResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDesiredPosj-response)))
  "Returns string type for a service object of type 'GetDesiredPosj-response"
  "dsr_msgs/GetDesiredPosjResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDesiredPosj-response>)))
  "Returns md5sum for a message object of type '<GetDesiredPosj-response>"
  "3d228bb409a5fe0306e199313e3ed277")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDesiredPosj-response)))
  "Returns md5sum for a message object of type 'GetDesiredPosj-response"
  "3d228bb409a5fe0306e199313e3ed277")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDesiredPosj-response>)))
  "Returns full string definition for message of type '<GetDesiredPosj-response>"
  (cl:format cl:nil "float64[6] pos               # joint pos(posj)  ~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDesiredPosj-response)))
  "Returns full string definition for message of type 'GetDesiredPosj-response"
  (cl:format cl:nil "float64[6] pos               # joint pos(posj)  ~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDesiredPosj-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDesiredPosj-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDesiredPosj-response
    (cl:cons ':pos (pos msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetDesiredPosj)))
  'GetDesiredPosj-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetDesiredPosj)))
  'GetDesiredPosj-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDesiredPosj)))
  "Returns string type for a service object of type '<GetDesiredPosj>"
  "dsr_msgs/GetDesiredPosj")