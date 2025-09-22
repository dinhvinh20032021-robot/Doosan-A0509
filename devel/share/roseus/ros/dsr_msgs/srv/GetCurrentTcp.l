;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetCurrentTcp)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetCurrentTcp (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETCURRENTTCP")
  (make-package "DSR_MSGS::GETCURRENTTCP"))
(unless (find-package "DSR_MSGS::GETCURRENTTCPREQUEST")
  (make-package "DSR_MSGS::GETCURRENTTCPREQUEST"))
(unless (find-package "DSR_MSGS::GETCURRENTTCPRESPONSE")
  (make-package "DSR_MSGS::GETCURRENTTCPRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetCurrentTcpRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetCurrentTcpRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass dsr_msgs::GetCurrentTcpResponse
  :super ros::object
  :slots (_info _success ))

(defmethod dsr_msgs::GetCurrentTcpResponse
  (:init
   (&key
    ((:info __info) "")
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _info (string __info))
   (setq _success __success)
   self)
  (:info
   (&optional __info)
   (if __info (setq _info __info)) _info)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; string _info
    4 (length _info)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _info
       (write-long (length _info) s) (princ _info s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _info
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _info (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetCurrentTcp
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetCurrentTcp :md5sum-) "4ed9e7e2e2815d295b7f655584191cad")
(setf (get dsr_msgs::GetCurrentTcp :datatype-) "dsr_msgs/GetCurrentTcp")
(setf (get dsr_msgs::GetCurrentTcp :request) dsr_msgs::GetCurrentTcpRequest)
(setf (get dsr_msgs::GetCurrentTcp :response) dsr_msgs::GetCurrentTcpResponse)

(defmethod dsr_msgs::GetCurrentTcpRequest
  (:response () (instance dsr_msgs::GetCurrentTcpResponse :init)))

(setf (get dsr_msgs::GetCurrentTcpRequest :md5sum-) "4ed9e7e2e2815d295b7f655584191cad")
(setf (get dsr_msgs::GetCurrentTcpRequest :datatype-) "dsr_msgs/GetCurrentTcpRequest")
(setf (get dsr_msgs::GetCurrentTcpRequest :definition-)
      "#____________________________________________________________________________________________
# get_current_tcp  
# It is the service to get the currently set TCP information from the robot controller
#____________________________________________________________________________________________

---
string         info # tcp name
bool        success
")

(setf (get dsr_msgs::GetCurrentTcpResponse :md5sum-) "4ed9e7e2e2815d295b7f655584191cad")
(setf (get dsr_msgs::GetCurrentTcpResponse :datatype-) "dsr_msgs/GetCurrentTcpResponse")
(setf (get dsr_msgs::GetCurrentTcpResponse :definition-)
      "#____________________________________________________________________________________________
# get_current_tcp  
# It is the service to get the currently set TCP information from the robot controller
#____________________________________________________________________________________________

---
string         info # tcp name
bool        success
")



(provide :dsr_msgs/GetCurrentTcp "4ed9e7e2e2815d295b7f655584191cad")


