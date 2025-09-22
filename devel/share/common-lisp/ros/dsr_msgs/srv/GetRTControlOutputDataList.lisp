; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude GetRTControlOutputDataList-request.msg.html

(cl:defclass <GetRTControlOutputDataList-request> (roslisp-msg-protocol:ros-message)
  ((version
    :reader version
    :initarg :version
    :type cl:string
    :initform ""))
)

(cl:defclass GetRTControlOutputDataList-request (<GetRTControlOutputDataList-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRTControlOutputDataList-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRTControlOutputDataList-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetRTControlOutputDataList-request> is deprecated: use dsr_msgs-srv:GetRTControlOutputDataList-request instead.")))

(cl:ensure-generic-function 'version-val :lambda-list '(m))
(cl:defmethod version-val ((m <GetRTControlOutputDataList-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:version-val is deprecated.  Use dsr_msgs-srv:version instead.")
  (version m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRTControlOutputDataList-request>) ostream)
  "Serializes a message object of type '<GetRTControlOutputDataList-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'version))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRTControlOutputDataList-request>) istream)
  "Deserializes a message object of type '<GetRTControlOutputDataList-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRTControlOutputDataList-request>)))
  "Returns string type for a service object of type '<GetRTControlOutputDataList-request>"
  "dsr_msgs/GetRTControlOutputDataListRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRTControlOutputDataList-request)))
  "Returns string type for a service object of type 'GetRTControlOutputDataList-request"
  "dsr_msgs/GetRTControlOutputDataListRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRTControlOutputDataList-request>)))
  "Returns md5sum for a message object of type '<GetRTControlOutputDataList-request>"
  "af5686a9e1b1a378bc0983d164821af1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRTControlOutputDataList-request)))
  "Returns md5sum for a message object of type 'GetRTControlOutputDataList-request"
  "af5686a9e1b1a378bc0983d164821af1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRTControlOutputDataList-request>)))
  "Returns full string definition for message of type '<GetRTControlOutputDataList-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_rt_control_output_data_list~%#____________________________________________________________________________________________~%~%string     version~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRTControlOutputDataList-request)))
  "Returns full string definition for message of type 'GetRTControlOutputDataList-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# get_rt_control_output_data_list~%#____________________________________________________________________________________________~%~%string     version~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRTControlOutputDataList-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'version))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRTControlOutputDataList-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRTControlOutputDataList-request
    (cl:cons ':version (version msg))
))
;//! \htmlinclude GetRTControlOutputDataList-response.msg.html

(cl:defclass <GetRTControlOutputDataList-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (data
    :reader data
    :initarg :data
    :type cl:string
    :initform ""))
)

(cl:defclass GetRTControlOutputDataList-response (<GetRTControlOutputDataList-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRTControlOutputDataList-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRTControlOutputDataList-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<GetRTControlOutputDataList-response> is deprecated: use dsr_msgs-srv:GetRTControlOutputDataList-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetRTControlOutputDataList-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <GetRTControlOutputDataList-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:data-val is deprecated.  Use dsr_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRTControlOutputDataList-response>) ostream)
  "Serializes a message object of type '<GetRTControlOutputDataList-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRTControlOutputDataList-response>) istream)
  "Deserializes a message object of type '<GetRTControlOutputDataList-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRTControlOutputDataList-response>)))
  "Returns string type for a service object of type '<GetRTControlOutputDataList-response>"
  "dsr_msgs/GetRTControlOutputDataListResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRTControlOutputDataList-response)))
  "Returns string type for a service object of type 'GetRTControlOutputDataList-response"
  "dsr_msgs/GetRTControlOutputDataListResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRTControlOutputDataList-response>)))
  "Returns md5sum for a message object of type '<GetRTControlOutputDataList-response>"
  "af5686a9e1b1a378bc0983d164821af1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRTControlOutputDataList-response)))
  "Returns md5sum for a message object of type 'GetRTControlOutputDataList-response"
  "af5686a9e1b1a378bc0983d164821af1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRTControlOutputDataList-response>)))
  "Returns full string definition for message of type '<GetRTControlOutputDataList-response>"
  (cl:format cl:nil "bool       success~%string     data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRTControlOutputDataList-response)))
  "Returns full string definition for message of type 'GetRTControlOutputDataList-response"
  (cl:format cl:nil "bool       success~%string     data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRTControlOutputDataList-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRTControlOutputDataList-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRTControlOutputDataList-response
    (cl:cons ':success (success msg))
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetRTControlOutputDataList)))
  'GetRTControlOutputDataList-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetRTControlOutputDataList)))
  'GetRTControlOutputDataList-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRTControlOutputDataList)))
  "Returns string type for a service object of type '<GetRTControlOutputDataList>"
  "dsr_msgs/GetRTControlOutputDataList")