; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude MoveHome-request.msg.html

(cl:defclass <MoveHome-request> (roslisp-msg-protocol:ros-message)
  ((target
    :reader target
    :initarg :target
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MoveHome-request (<MoveHome-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveHome-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveHome-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<MoveHome-request> is deprecated: use dsr_msgs-srv:MoveHome-request instead.")))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <MoveHome-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:target-val is deprecated.  Use dsr_msgs-srv:target instead.")
  (target m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveHome-request>) ostream)
  "Serializes a message object of type '<MoveHome-request>"
  (cl:let* ((signed (cl:slot-value msg 'target)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveHome-request>) istream)
  "Deserializes a message object of type '<MoveHome-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveHome-request>)))
  "Returns string type for a service object of type '<MoveHome-request>"
  "dsr_msgs/MoveHomeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveHome-request)))
  "Returns string type for a service object of type 'MoveHome-request"
  "dsr_msgs/MoveHomeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveHome-request>)))
  "Returns md5sum for a message object of type '<MoveHome-request>"
  "1df045e08f5aa30b46232621f50af2fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveHome-request)))
  "Returns md5sum for a message object of type 'MoveHome-request"
  "1df045e08f5aa30b46232621f50af2fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveHome-request>)))
  "Returns full string definition for message of type '<MoveHome-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# move_home~%# Homing is performed by moving to the joint motion to the mechanical or user defined home position.~%# According to the input parameter [target], it moves to the mechanical home defined in the system or the home set by the user.~%#____________________________________________________________________________________________~%~%int8       target           # DR_HOME_TARGET_MECHANIC(0) : Mechanical home, joint angle (0,0,0,0,0,0)~%                            # DR_HOME_TARGET_USER(1)     : user home~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveHome-request)))
  "Returns full string definition for message of type 'MoveHome-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# move_home~%# Homing is performed by moving to the joint motion to the mechanical or user defined home position.~%# According to the input parameter [target], it moves to the mechanical home defined in the system or the home set by the user.~%#____________________________________________________________________________________________~%~%int8       target           # DR_HOME_TARGET_MECHANIC(0) : Mechanical home, joint angle (0,0,0,0,0,0)~%                            # DR_HOME_TARGET_USER(1)     : user home~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveHome-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveHome-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveHome-request
    (cl:cons ':target (target msg))
))
;//! \htmlinclude MoveHome-response.msg.html

(cl:defclass <MoveHome-response> (roslisp-msg-protocol:ros-message)
  ((res
    :reader res
    :initarg :res
    :type cl:fixnum
    :initform 0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoveHome-response (<MoveHome-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveHome-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveHome-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<MoveHome-response> is deprecated: use dsr_msgs-srv:MoveHome-response instead.")))

(cl:ensure-generic-function 'res-val :lambda-list '(m))
(cl:defmethod res-val ((m <MoveHome-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:res-val is deprecated.  Use dsr_msgs-srv:res instead.")
  (res m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <MoveHome-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveHome-response>) ostream)
  "Serializes a message object of type '<MoveHome-response>"
  (cl:let* ((signed (cl:slot-value msg 'res)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveHome-response>) istream)
  "Deserializes a message object of type '<MoveHome-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'res) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveHome-response>)))
  "Returns string type for a service object of type '<MoveHome-response>"
  "dsr_msgs/MoveHomeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveHome-response)))
  "Returns string type for a service object of type 'MoveHome-response"
  "dsr_msgs/MoveHomeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveHome-response>)))
  "Returns md5sum for a message object of type '<MoveHome-response>"
  "1df045e08f5aa30b46232621f50af2fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveHome-response)))
  "Returns md5sum for a message object of type 'MoveHome-response"
  "1df045e08f5aa30b46232621f50af2fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveHome-response>)))
  "Returns full string definition for message of type '<MoveHome-response>"
  (cl:format cl:nil "int8       res              # 0=success, otherwise fail ~%bool       success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveHome-response)))
  "Returns full string definition for message of type 'MoveHome-response"
  (cl:format cl:nil "int8       res              # 0=success, otherwise fail ~%bool       success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveHome-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveHome-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveHome-response
    (cl:cons ':res (res msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveHome)))
  'MoveHome-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveHome)))
  'MoveHome-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveHome)))
  "Returns string type for a service object of type '<MoveHome>"
  "dsr_msgs/MoveHome")