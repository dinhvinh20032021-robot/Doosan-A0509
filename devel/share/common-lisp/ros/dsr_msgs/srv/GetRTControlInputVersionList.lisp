; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetRTControlInputVersionList-request.msg.html

(cl:defclass <GetRTControlInputVersionList-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetRTControlInputVersionList-request (<GetRTControlInputVersionList-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRTControlInputVersionList-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRTControlInputVersionList-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetRTControlInputVersionList-request> is deprecated: use dsr_msgs-srv:GetRTControlInputVersionList-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRTControlInputVersionList-request>) ostream)
  "Serializes a message object of type '<GetRTControlInputVersionList-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRTControlInputVersionList-request>) istream)
  "Deserializes a message object of type '<GetRTControlInputVersionList-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRTControlInputVersionList-request>)))
  "Returns string type for a service object of type '<GetRTControlInputVersionList-request>"
  "dsr_msgs/GetRTControlInputVersionListRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRTControlInputVersionList-request)))
  "Returns string type for a service object of type 'GetRTControlInputVersionList-request"
  "dsr_msgs/GetRTControlInputVersionListRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRTControlInputVersionList-request>)))
  "Returns md5sum for a message object of type '<GetRTControlInputVersionList-request>"
  "621ab4fad3b2f7679edb9817178ccd57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRTControlInputVersionList-request)))
  "Returns md5sum for a message object of type 'GetRTControlInputVersionList-request"
  "621ab4fad3b2f7679edb9817178ccd57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRTControlInputVersionList-request>)))
  "Returns full string definition for message of type '<GetRTControlInputVersionList-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_rt_control_input_version_list~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRTControlInputVersionList-request)))
  "Returns full string definition for message of type 'GetRTControlInputVersionList-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_rt_control_input_version_list~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRTControlInputVersionList-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRTControlInputVersionList-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRTControlInputVersionList-request
))
;//! \htmlinclude GetRTControlInputVersionList-response.msg.html

(cl:defclass <GetRTControlInputVersionList-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (version
    :reader version
    :initarg :version
    :type cl:string
    :initform ""))
)

(cl:defclass GetRTControlInputVersionList-response (<GetRTControlInputVersionList-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRTControlInputVersionList-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRTControlInputVersionList-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetRTControlInputVersionList-response> is deprecated: use dsr_msgs-srv:GetRTControlInputVersionList-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetRTControlInputVersionList-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'version-val :lambda-list '(m))
(cl:defmethod version-val ((m <GetRTControlInputVersionList-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:version-val is deprecated.  Use dsr_msgs-srv:version instead.")
  (version m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRTControlInputVersionList-response>) ostream)
  "Serializes a message object of type '<GetRTControlInputVersionList-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'version))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRTControlInputVersionList-response>) istream)
  "Deserializes a message object of type '<GetRTControlInputVersionList-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'version) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'version) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRTControlInputVersionList-response>)))
  "Returns string type for a service object of type '<GetRTControlInputVersionList-response>"
  "dsr_msgs/GetRTControlInputVersionListResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRTControlInputVersionList-response)))
  "Returns string type for a service object of type 'GetRTControlInputVersionList-response"
  "dsr_msgs/GetRTControlInputVersionListResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRTControlInputVersionList-response>)))
  "Returns md5sum for a message object of type '<GetRTControlInputVersionList-response>"
  "621ab4fad3b2f7679edb9817178ccd57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRTControlInputVersionList-response)))
  "Returns md5sum for a message object of type 'GetRTControlInputVersionList-response"
  "621ab4fad3b2f7679edb9817178ccd57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRTControlInputVersionList-response>)))
  "Returns full string definition for message of type '<GetRTControlInputVersionList-response>"
  (cl:format cl:nil "bool       success~%string     version~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRTControlInputVersionList-response)))
  "Returns full string definition for message of type 'GetRTControlInputVersionList-response"
  (cl:format cl:nil "bool       success~%string     version~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRTControlInputVersionList-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'version))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRTControlInputVersionList-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRTControlInputVersionList-response
    (cl:cons ':success (success msg))
    (cl:cons ':version (version msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetRTControlInputVersionList)))
  'GetRTControlInputVersionList-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetRTControlInputVersionList)))
  'GetRTControlInputVersionList-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRTControlInputVersionList)))
  "Returns string type for a service object of type '<GetRTControlInputVersionList>"
  "dsr_msgs/GetRTControlInputVersionList")