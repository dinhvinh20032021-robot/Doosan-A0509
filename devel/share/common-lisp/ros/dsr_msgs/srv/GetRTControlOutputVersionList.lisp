; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetRTControlOutputVersionList-request.msg.html

(cl:defclass <GetRTControlOutputVersionList-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetRTControlOutputVersionList-request (<GetRTControlOutputVersionList-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRTControlOutputVersionList-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRTControlOutputVersionList-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetRTControlOutputVersionList-request> is deprecated: use dsr_msgs-srv:GetRTControlOutputVersionList-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRTControlOutputVersionList-request>) ostream)
  "Serializes a message object of type '<GetRTControlOutputVersionList-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRTControlOutputVersionList-request>) istream)
  "Deserializes a message object of type '<GetRTControlOutputVersionList-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRTControlOutputVersionList-request>)))
  "Returns string type for a service object of type '<GetRTControlOutputVersionList-request>"
  "dsr_msgs/GetRTControlOutputVersionListRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRTControlOutputVersionList-request)))
  "Returns string type for a service object of type 'GetRTControlOutputVersionList-request"
  "dsr_msgs/GetRTControlOutputVersionListRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRTControlOutputVersionList-request>)))
  "Returns md5sum for a message object of type '<GetRTControlOutputVersionList-request>"
  "621ab4fad3b2f7679edb9817178ccd57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRTControlOutputVersionList-request)))
  "Returns md5sum for a message object of type 'GetRTControlOutputVersionList-request"
  "621ab4fad3b2f7679edb9817178ccd57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRTControlOutputVersionList-request>)))
  "Returns full string definition for message of type '<GetRTControlOutputVersionList-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_rt_control_output_version_list~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRTControlOutputVersionList-request)))
  "Returns full string definition for message of type 'GetRTControlOutputVersionList-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_rt_control_output_version_list~%#____________________________________________________________________________________________~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRTControlOutputVersionList-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRTControlOutputVersionList-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRTControlOutputVersionList-request
))
;//! \htmlinclude GetRTControlOutputVersionList-response.msg.html

(cl:defclass <GetRTControlOutputVersionList-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetRTControlOutputVersionList-response (<GetRTControlOutputVersionList-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRTControlOutputVersionList-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRTControlOutputVersionList-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetRTControlOutputVersionList-response> is deprecated: use dsr_msgs-srv:GetRTControlOutputVersionList-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetRTControlOutputVersionList-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'version-val :lambda-list '(m))
(cl:defmethod version-val ((m <GetRTControlOutputVersionList-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:version-val is deprecated.  Use dsr_msgs-srv:version instead.")
  (version m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRTControlOutputVersionList-response>) ostream)
  "Serializes a message object of type '<GetRTControlOutputVersionList-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'version))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRTControlOutputVersionList-response>) istream)
  "Deserializes a message object of type '<GetRTControlOutputVersionList-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRTControlOutputVersionList-response>)))
  "Returns string type for a service object of type '<GetRTControlOutputVersionList-response>"
  "dsr_msgs/GetRTControlOutputVersionListResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRTControlOutputVersionList-response)))
  "Returns string type for a service object of type 'GetRTControlOutputVersionList-response"
  "dsr_msgs/GetRTControlOutputVersionListResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRTControlOutputVersionList-response>)))
  "Returns md5sum for a message object of type '<GetRTControlOutputVersionList-response>"
  "621ab4fad3b2f7679edb9817178ccd57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRTControlOutputVersionList-response)))
  "Returns md5sum for a message object of type 'GetRTControlOutputVersionList-response"
  "621ab4fad3b2f7679edb9817178ccd57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRTControlOutputVersionList-response>)))
  "Returns full string definition for message of type '<GetRTControlOutputVersionList-response>"
  (cl:format cl:nil "bool       success~%string     version~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRTControlOutputVersionList-response)))
  "Returns full string definition for message of type 'GetRTControlOutputVersionList-response"
  (cl:format cl:nil "bool       success~%string     version~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRTControlOutputVersionList-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'version))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRTControlOutputVersionList-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRTControlOutputVersionList-response
    (cl:cons ':success (success msg))
    (cl:cons ':version (version msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetRTControlOutputVersionList)))
  'GetRTControlOutputVersionList-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetRTControlOutputVersionList)))
  'GetRTControlOutputVersionList-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRTControlOutputVersionList)))
  "Returns string type for a service object of type '<GetRTControlOutputVersionList>"
  "dsr_msgs/GetRTControlOutputVersionList")