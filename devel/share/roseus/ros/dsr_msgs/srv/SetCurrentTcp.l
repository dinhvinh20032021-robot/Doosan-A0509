;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetCurrentTcp)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetCurrentTcp (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETCURRENTTCP")
  (make-package "DSR_MSGS::SETCURRENTTCP"))
(unless (find-package "DSR_MSGS::SETCURRENTTCPREQUEST")
  (make-package "DSR_MSGS::SETCURRENTTCPREQUEST"))
(unless (find-package "DSR_MSGS::SETCURRENTTCPRESPONSE")
  (make-package "DSR_MSGS::SETCURRENTTCPRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetCurrentTcpRequest
  :super ros::object
  :slots (_name ))

(defmethod dsr_msgs::SetCurrentTcpRequest
  (:init
   (&key
    ((:name __name) "")
    )
   (send-super :init)
   (setq _name (string __name))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass dsr_msgs::SetCurrentTcpResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetCurrentTcpResponse
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

(defclass dsr_msgs::SetCurrentTcp
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetCurrentTcp :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::SetCurrentTcp :datatype-) "dsr_msgs/SetCurrentTcp")
(setf (get dsr_msgs::SetCurrentTcp :request) dsr_msgs::SetCurrentTcpRequest)
(setf (get dsr_msgs::SetCurrentTcp :response) dsr_msgs::SetCurrentTcpResponse)

(defmethod dsr_msgs::SetCurrentTcpRequest
  (:response () (instance dsr_msgs::SetCurrentTcpResponse :init)))

(setf (get dsr_msgs::SetCurrentTcpRequest :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::SetCurrentTcpRequest :datatype-) "dsr_msgs/SetCurrentTcpRequest")
(setf (get dsr_msgs::SetCurrentTcpRequest :definition-)
      "#____________________________________________________________________________________________
# set_current_tcp  
# It is a service that sets the information about the currently installed TCP
#____________________________________________________________________________________________

string         name # tcp name
---
bool           success
")

(setf (get dsr_msgs::SetCurrentTcpResponse :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::SetCurrentTcpResponse :datatype-) "dsr_msgs/SetCurrentTcpResponse")
(setf (get dsr_msgs::SetCurrentTcpResponse :definition-)
      "#____________________________________________________________________________________________
# set_current_tcp  
# It is a service that sets the information about the currently installed TCP
#____________________________________________________________________________________________

string         name # tcp name
---
bool           success
")



(provide :dsr_msgs/SetCurrentTcp "d08a3b641c2f8680fbdfb1ea2e17a3e1")


