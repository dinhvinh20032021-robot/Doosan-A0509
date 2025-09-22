; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-srv)


;//! \htmlinclude SerialSendData-request.msg.html

(cl:defclass <SerialSendData-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:string
    :initform ""))
)

(cl:defclass SerialSendData-request (<SerialSendData-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SerialSendData-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SerialSendData-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SerialSendData-request> is deprecated: use dsr_msgs-srv:SerialSendData-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SerialSendData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:data-val is deprecated.  Use dsr_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SerialSendData-request>) ostream)
  "Serializes a message object of type '<SerialSendData-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SerialSendData-request>) istream)
  "Deserializes a message object of type '<SerialSendData-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SerialSendData-request>)))
  "Returns string type for a service object of type '<SerialSendData-request>"
  "dsr_msgs/SerialSendDataRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SerialSendData-request)))
  "Returns string type for a service object of type 'SerialSendData-request"
  "dsr_msgs/SerialSendDataRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SerialSendData-request>)))
  "Returns md5sum for a message object of type '<SerialSendData-request>"
  "e7ac98ca304b04e45988c61adb6bc4ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SerialSendData-request)))
  "Returns md5sum for a message object of type 'SerialSendData-request"
  "e7ac98ca304b04e45988c61adb6bc4ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SerialSendData-request>)))
  "Returns full string definition for message of type '<SerialSendData-request>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# serial_send_data~%# send byte data to other device ~%# you can operate the real-robotiq gripper by modbus rtu~%#____________________________________________________________________________________________~%~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SerialSendData-request)))
  "Returns full string definition for message of type 'SerialSendData-request"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# serial_send_data~%# send byte data to other device ~%# you can operate the real-robotiq gripper by modbus rtu~%#____________________________________________________________________________________________~%~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SerialSendData-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SerialSendData-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SerialSendData-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude SerialSendData-response.msg.html

(cl:defclass <SerialSendData-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SerialSendData-response (<SerialSendData-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SerialSendData-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SerialSendData-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-srv:<SerialSendData-response> is deprecated: use dsr_msgs-srv:SerialSendData-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SerialSendData-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-srv:success-val is deprecated.  Use dsr_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SerialSendData-response>) ostream)
  "Serializes a message object of type '<SerialSendData-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SerialSendData-response>) istream)
  "Deserializes a message object of type '<SerialSendData-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SerialSendData-response>)))
  "Returns string type for a service object of type '<SerialSendData-response>"
  "dsr_msgs/SerialSendDataResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SerialSendData-response)))
  "Returns string type for a service object of type 'SerialSendData-response"
  "dsr_msgs/SerialSendDataResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SerialSendData-response>)))
  "Returns md5sum for a message object of type '<SerialSendData-response>"
  "e7ac98ca304b04e45988c61adb6bc4ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SerialSendData-response)))
  "Returns md5sum for a message object of type 'SerialSendData-response"
  "e7ac98ca304b04e45988c61adb6bc4ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SerialSendData-response>)))
  "Returns full string definition for message of type '<SerialSendData-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SerialSendData-response)))
  "Returns full string definition for message of type 'SerialSendData-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SerialSendData-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SerialSendData-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SerialSendData-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SerialSendData)))
  'SerialSendData-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SerialSendData)))
  'SerialSendData-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SerialSendData)))
  "Returns string type for a service object of type '<SerialSendData>"
  "dsr_msgs/SerialSendData")