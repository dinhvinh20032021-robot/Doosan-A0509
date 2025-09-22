; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetDesiredVelj-request.msg.html

(cl:defclass <GetDesiredVelj-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetDesiredVelj-request (<GetDesiredVelj-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDesiredVelj-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDesiredVelj-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetDesiredVelj-request> is deprecated: use dsr_msgs-srv:GetDesiredVelj-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDesiredVelj-request>) ostream)
  "Serializes a message object of type '<GetDesiredVelj-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDesiredVelj-request>) istream)
  "Deserializes a message object of type '<GetDesiredVelj-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDesiredVelj-request>)))
  "Returns string type for a service object of type '<GetDesiredVelj-request>"
  "dsr_msgs/GetDesiredVeljRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDesiredVelj-request)))
  "Returns string type for a service object of type 'GetDesiredVelj-request"
  "dsr_msgs/GetDesiredVeljRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDesiredVelj-request>)))
  "Returns md5sum for a message object of type '<GetDesiredVelj-request>"
  "88bf0c6c29b23cd493e33b3354d4f21a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDesiredVelj-request)))
  "Returns md5sum for a message object of type 'GetDesiredVelj-request"
  "88bf0c6c29b23cd493e33b3354d4f21a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDesiredVelj-request>)))
  "Returns full string definition for message of type '<GetDesiredVelj-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_desired_velj()  ~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDesiredVelj-request)))
  "Returns full string definition for message of type 'GetDesiredVelj-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_desired_velj()  ~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDesiredVelj-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDesiredVelj-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDesiredVelj-request
))
;//! \htmlinclude GetDesiredVelj-response.msg.html

(cl:defclass <GetDesiredVelj-response> (roslisp-msg-protocol:ros-message)
  ((joint_vel
    :reader joint_vel
    :initarg :joint_vel
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetDesiredVelj-response (<GetDesiredVelj-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDesiredVelj-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDesiredVelj-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetDesiredVelj-response> is deprecated: use dsr_msgs-srv:GetDesiredVelj-response instead.")))

(cl:ensure-generic-function 'joint_vel-val :lambda-list '(m))
(cl:defmethod joint_vel-val ((m <GetDesiredVelj-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:joint_vel-val is deprecated.  Use dsr_msgs-srv:joint_vel instead.")
  (joint_vel m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetDesiredVelj-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDesiredVelj-response>) ostream)
  "Serializes a message object of type '<GetDesiredVelj-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'joint_vel))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDesiredVelj-response>) istream)
  "Deserializes a message object of type '<GetDesiredVelj-response>"
  (cl:setf (cl:slot-value msg 'joint_vel) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'joint_vel)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDesiredVelj-response>)))
  "Returns string type for a service object of type '<GetDesiredVelj-response>"
  "dsr_msgs/GetDesiredVeljResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDesiredVelj-response)))
  "Returns string type for a service object of type 'GetDesiredVelj-response"
  "dsr_msgs/GetDesiredVeljResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDesiredVelj-response>)))
  "Returns md5sum for a message object of type '<GetDesiredVelj-response>"
  "88bf0c6c29b23cd493e33b3354d4f21a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDesiredVelj-response)))
  "Returns md5sum for a message object of type 'GetDesiredVelj-response"
  "88bf0c6c29b23cd493e33b3354d4f21a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDesiredVelj-response>)))
  "Returns full string definition for message of type '<GetDesiredVelj-response>"
  (cl:format cl:nil "float64[6] joint_vel               # Target joint velocity ~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDesiredVelj-response)))
  "Returns full string definition for message of type 'GetDesiredVelj-response"
  (cl:format cl:nil "float64[6] joint_vel               # Target joint velocity ~%bool       success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDesiredVelj-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_vel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDesiredVelj-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDesiredVelj-response
    (cl:cons ':joint_vel (joint_vel msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetDesiredVelj)))
  'GetDesiredVelj-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetDesiredVelj)))
  'GetDesiredVelj-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDesiredVelj)))
  "Returns string type for a service object of type '<GetDesiredVelj>"
  "dsr_msgs/GetDesiredVelj")