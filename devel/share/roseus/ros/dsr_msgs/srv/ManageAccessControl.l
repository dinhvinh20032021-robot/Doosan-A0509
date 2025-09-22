;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ManageAccessControl)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ManageAccessControl (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::MANAGEACCESSCONTROL")
  (make-package "DSR_MSGS::MANAGEACCESSCONTROL"))
(unless (find-package "DSR_MSGS::MANAGEACCESSCONTROLREQUEST")
  (make-package "DSR_MSGS::MANAGEACCESSCONTROLREQUEST"))
(unless (find-package "DSR_MSGS::MANAGEACCESSCONTROLRESPONSE")
  (make-package "DSR_MSGS::MANAGEACCESSCONTROLRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::ManageAccessControlRequest
  :super ros::object
  :slots (_access_control ))

(defmethod dsr_msgs::ManageAccessControlRequest
  (:init
   (&key
    ((:access_control __access_control) 0)
    )
   (send-super :init)
   (setq _access_control (round __access_control))
   self)
  (:access_control
   (&optional __access_control)
   (if __access_control (setq _access_control __access_control)) _access_control)
  (:serialization-length
   ()
   (+
    ;; int8 _access_control
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _access_control
       (write-byte _access_control s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _access_control
     (setq _access_control (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _access_control 127) (setq _access_control (- _access_control 256)))
   ;;
   self)
  )

(defclass dsr_msgs::ManageAccessControlResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::ManageAccessControlResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::ManageAccessControl
  :super ros::object
  :slots ())

(setf (get dsr_msgs::ManageAccessControl :md5sum-) "73b144debe54a1756145c8c37ae08b64")
(setf (get dsr_msgs::ManageAccessControl :datatype-) "dsr_msgs/ManageAccessControl")
(setf (get dsr_msgs::ManageAccessControl :request) dsr_msgs::ManageAccessControlRequest)
(setf (get dsr_msgs::ManageAccessControl :response) dsr_msgs::ManageAccessControlResponse)

(defmethod dsr_msgs::ManageAccessControlRequest
  (:response () (instance dsr_msgs::ManageAccessControlResponse :init)))

(setf (get dsr_msgs::ManageAccessControlRequest :md5sum-) "73b144debe54a1756145c8c37ae08b64")
(setf (get dsr_msgs::ManageAccessControlRequest :datatype-) "dsr_msgs/ManageAccessControlRequest")
(setf (get dsr_msgs::ManageAccessControlRequest :definition-)
      "#____________________________________________________________________________________________
# manage_access_control
#____________________________________________________________________________________________

int8 access_control  # 0 : MANAGE_ACCESS_CONTROL_FORCE_REQUEST
                     # 1 : MANAGE_ACCESS_CONTROL_REQUEST
                     # 2 : MANAGE_ACCESS_CONTROL_RESPONSE_YES
                     # 3 : MANAGE_ACCESS_CONTROL_RESPONSE_NO
---
bool success

")

(setf (get dsr_msgs::ManageAccessControlResponse :md5sum-) "73b144debe54a1756145c8c37ae08b64")
(setf (get dsr_msgs::ManageAccessControlResponse :datatype-) "dsr_msgs/ManageAccessControlResponse")
(setf (get dsr_msgs::ManageAccessControlResponse :definition-)
      "#____________________________________________________________________________________________
# manage_access_control
#____________________________________________________________________________________________

int8 access_control  # 0 : MANAGE_ACCESS_CONTROL_FORCE_REQUEST
                     # 1 : MANAGE_ACCESS_CONTROL_REQUEST
                     # 2 : MANAGE_ACCESS_CONTROL_RESPONSE_YES
                     # 3 : MANAGE_ACCESS_CONTROL_RESPONSE_NO
---
bool success

")



(provide :dsr_msgs/ManageAccessControl "73b144debe54a1756145c8c37ae08b64")


