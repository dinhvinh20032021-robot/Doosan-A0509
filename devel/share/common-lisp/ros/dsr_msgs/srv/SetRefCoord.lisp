; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SetRefCoord-request.msg.html

(cl:defclass <SetRefCoord-request> (roslisp-msg-protocol:ros-message)
  ((coord
    :reader coord
    :initarg :coord
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetRefCoord-request (<SetRefCoord-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRefCoord-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRefCoord-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetRefCoord-request> is deprecated: use dsr_msgs-srv:SetRefCoord-request instead.")))

(cl:ensure-generic-function 'coord-val :lambda-list '(m))
(cl:defmethod coord-val ((m <SetRefCoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:coord-val is deprecated.  Use dsr_msgs-srv:coord instead.")
  (coord m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRefCoord-request>) ostream)
  "Serializes a message object of type '<SetRefCoord-request>"
  (cl:let* ((signed (cl:slot-value msg 'coord)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRefCoord-request>) istream)
  "Deserializes a message object of type '<SetRefCoord-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'coord) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRefCoord-request>)))
  "Returns string type for a service object of type '<SetRefCoord-request>"
  "dsr_msgs/SetRefCoordRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRefCoord-request)))
  "Returns string type for a service object of type 'SetRefCoord-request"
  "dsr_msgs/SetRefCoordRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRefCoord-request>)))
  "Returns md5sum for a message object of type '<SetRefCoord-request>"
  "1e3275b6f46c8260502524b4e6bedf9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRefCoord-request)))
  "Returns md5sum for a message object of type 'SetRefCoord-request"
  "1e3275b6f46c8260502524b4e6bedf9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRefCoord-request>)))
  "Returns full string definition for message of type '<SetRefCoord-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_ref_coord ~%#____________________________________________________________________________________________~%~%int8       coord            # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~~200)~%                            # <DR_WORLD is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRefCoord-request)))
  "Returns full string definition for message of type 'SetRefCoord-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# set_ref_coord ~%#____________________________________________________________________________________________~%~%int8       coord            # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~~200)~%                            # <DR_WORLD is only available in M2.40 or later> ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRefCoord-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRefCoord-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRefCoord-request
    (cl:cons ':coord (coord msg))
))
;//! \htmlinclude SetRefCoord-response.msg.html

(cl:defclass <SetRefCoord-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetRefCoord-response (<SetRefCoord-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRefCoord-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRefCoord-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SetRefCoord-response> is deprecated: use dsr_msgs-srv:SetRefCoord-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetRefCoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRefCoord-response>) ostream)
  "Serializes a message object of type '<SetRefCoord-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRefCoord-response>) istream)
  "Deserializes a message object of type '<SetRefCoord-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRefCoord-response>)))
  "Returns string type for a service object of type '<SetRefCoord-response>"
  "dsr_msgs/SetRefCoordResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRefCoord-response)))
  "Returns string type for a service object of type 'SetRefCoord-response"
  "dsr_msgs/SetRefCoordResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRefCoord-response>)))
  "Returns md5sum for a message object of type '<SetRefCoord-response>"
  "1e3275b6f46c8260502524b4e6bedf9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRefCoord-response)))
  "Returns md5sum for a message object of type 'SetRefCoord-response"
  "1e3275b6f46c8260502524b4e6bedf9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRefCoord-response>)))
  "Returns full string definition for message of type '<SetRefCoord-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRefCoord-response)))
  "Returns full string definition for message of type 'SetRefCoord-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRefCoord-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRefCoord-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRefCoord-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetRefCoord)))
  'SetRefCoord-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetRefCoord)))
  'SetRefCoord-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRefCoord)))
  "Returns string type for a service object of type '<SetRefCoord>"
  "dsr_msgs/SetRefCoord")