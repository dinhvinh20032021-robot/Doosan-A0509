;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ConfigDeleteTcp)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ConfigDeleteTcp (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::CONFIGDELETETCP")
  (make-package "DSR_MSGS::CONFIGDELETETCP"))
(unless (find-package "DSR_MSGS::CONFIGDELETETCPREQUEST")
  (make-package "DSR_MSGS::CONFIGDELETETCPREQUEST"))
(unless (find-package "DSR_MSGS::CONFIGDELETETCPRESPONSE")
  (make-package "DSR_MSGS::CONFIGDELETETCPRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::ConfigDeleteTcpRequest
  :super ros::object
  :slots (_name ))

(defmethod dsr_msgs::ConfigDeleteTcpRequest
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

(defclass dsr_msgs::ConfigDeleteTcpResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::ConfigDeleteTcpResponse
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

(defclass dsr_msgs::ConfigDeleteTcp
  :super ros::object
  :slots ())

(setf (get dsr_msgs::ConfigDeleteTcp :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::ConfigDeleteTcp :datatype-) "dsr_msgs/ConfigDeleteTcp")
(setf (get dsr_msgs::ConfigDeleteTcp :request) dsr_msgs::ConfigDeleteTcpRequest)
(setf (get dsr_msgs::ConfigDeleteTcp :response) dsr_msgs::ConfigDeleteTcpResponse)

(defmethod dsr_msgs::ConfigDeleteTcpRequest
  (:response () (instance dsr_msgs::ConfigDeleteTcpResponse :init)))

(setf (get dsr_msgs::ConfigDeleteTcpRequest :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::ConfigDeleteTcpRequest :datatype-) "dsr_msgs/ConfigDeleteTcpRequest")
(setf (get dsr_msgs::ConfigDeleteTcpRequest :definition-)
      "#____________________________________________________________________________________________
# config_delete_tcp  
# It is a service for deleting the TCP information registered in advance in the robot controller
#____________________________________________________________________________________________

string          name             # tcp name 
---
bool success
")

(setf (get dsr_msgs::ConfigDeleteTcpResponse :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::ConfigDeleteTcpResponse :datatype-) "dsr_msgs/ConfigDeleteTcpResponse")
(setf (get dsr_msgs::ConfigDeleteTcpResponse :definition-)
      "#____________________________________________________________________________________________
# config_delete_tcp  
# It is a service for deleting the TCP information registered in advance in the robot controller
#____________________________________________________________________________________________

string          name             # tcp name 
---
bool success
")



(provide :dsr_msgs/ConfigDeleteTcp "d08a3b641c2f8680fbdfb1ea2e17a3e1")


