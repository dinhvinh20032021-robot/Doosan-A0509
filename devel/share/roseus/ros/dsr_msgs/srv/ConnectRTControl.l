;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ConnectRTControl)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ConnectRTControl (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::CONNECTRTCONTROL")
  (make-package "DSR_MSGS::CONNECTRTCONTROL"))
(unless (find-package "DSR_MSGS::CONNECTRTCONTROLREQUEST")
  (make-package "DSR_MSGS::CONNECTRTCONTROLREQUEST"))
(unless (find-package "DSR_MSGS::CONNECTRTCONTROLRESPONSE")
  (make-package "DSR_MSGS::CONNECTRTCONTROLRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::ConnectRTControlRequest
  :super ros::object
  :slots (_ip_address _port ))

(defmethod dsr_msgs::ConnectRTControlRequest
  (:init
   (&key
    ((:ip_address __ip_address) "")
    ((:port __port) 0)
    )
   (send-super :init)
   (setq _ip_address (string __ip_address))
   (setq _port (round __port))
   self)
  (:ip_address
   (&optional __ip_address)
   (if __ip_address (setq _ip_address __ip_address)) _ip_address)
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:serialization-length
   ()
   (+
    ;; string _ip_address
    4 (length _ip_address)
    ;; uint32 _port
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _ip_address
       (write-long (length _ip_address) s) (princ _ip_address s)
     ;; uint32 _port
       (write-long _port s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _ip_address
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ip_address (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _port
     (setq _port (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dsr_msgs::ConnectRTControlResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::ConnectRTControlResponse
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

(defclass dsr_msgs::ConnectRTControl
  :super ros::object
  :slots ())

(setf (get dsr_msgs::ConnectRTControl :md5sum-) "924e165a726577b8d52c2048a873475b")
(setf (get dsr_msgs::ConnectRTControl :datatype-) "dsr_msgs/ConnectRTControl")
(setf (get dsr_msgs::ConnectRTControl :request) dsr_msgs::ConnectRTControlRequest)
(setf (get dsr_msgs::ConnectRTControl :response) dsr_msgs::ConnectRTControlResponse)

(defmethod dsr_msgs::ConnectRTControlRequest
  (:response () (instance dsr_msgs::ConnectRTControlResponse :init)))

(setf (get dsr_msgs::ConnectRTControlRequest :md5sum-) "924e165a726577b8d52c2048a873475b")
(setf (get dsr_msgs::ConnectRTControlRequest :datatype-) "dsr_msgs/ConnectRTControlRequest")
(setf (get dsr_msgs::ConnectRTControlRequest :definition-)
      "#____________________________________________________________________________________________
# connect_rt_control
#____________________________________________________________________________________________

string     ip_address
uint32     port
---
bool       success
")

(setf (get dsr_msgs::ConnectRTControlResponse :md5sum-) "924e165a726577b8d52c2048a873475b")
(setf (get dsr_msgs::ConnectRTControlResponse :datatype-) "dsr_msgs/ConnectRTControlResponse")
(setf (get dsr_msgs::ConnectRTControlResponse :definition-)
      "#____________________________________________________________________________________________
# connect_rt_control
#____________________________________________________________________________________________

string     ip_address
uint32     port
---
bool       success
")



(provide :dsr_msgs/ConnectRTControl "924e165a726577b8d52c2048a873475b")


