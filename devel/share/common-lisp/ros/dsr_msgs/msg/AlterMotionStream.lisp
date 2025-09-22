; Auto-generated. Do not edit!


(cl:in-package dsr_msgs-msg)


;//! \htmlinclude AlterMotionStream.msg.html

(cl:defclass <AlterMotionStream> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass AlterMotionStream (<AlterMotionStream>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AlterMotionStream>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AlterMotionStream)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dsr_msgs-msg:<AlterMotionStream> is deprecated: use dsr_msgs-msg:AlterMotionStream instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <AlterMotionStream>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dsr_msgs-msg:pos-val is deprecated.  Use dsr_msgs-msg:pos instead.")
  (pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AlterMotionStream>) ostream)
  "Serializes a message object of type '<AlterMotionStream>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AlterMotionStream>) istream)
  "Deserializes a message object of type '<AlterMotionStream>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AlterMotionStream>)))
  "Returns string type for a message object of type '<AlterMotionStream>"
  "dsr_msgs/AlterMotionStream")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AlterMotionStream)))
  "Returns string type for a message object of type 'AlterMotionStream"
  "dsr_msgs/AlterMotionStream")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AlterMotionStream>)))
  "Returns md5sum for a message object of type '<AlterMotionStream>"
  "537431324117c3d1847d70e057990155")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AlterMotionStream)))
  "Returns md5sum for a message object of type 'AlterMotionStream"
  "537431324117c3d1847d70e057990155")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AlterMotionStream>)))
  "Returns full string definition for message of type '<AlterMotionStream>"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# alter_motion  ~%# ~%#____________________________________________________________________________________________~%~%float64[6] pos               # position  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AlterMotionStream)))
  "Returns full string definition for message of type 'AlterMotionStream"
  (cl:format cl:nil "#____________________________________________________________________________________________~%# alter_motion  ~%# ~%#____________________________________________________________________________________________~%~%float64[6] pos               # position  ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AlterMotionStream>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AlterMotionStream>))
  "Converts a ROS message object to a list"
  (cl:list 'AlterMotionStream
    (cl:cons ':pos (pos msg))
))
