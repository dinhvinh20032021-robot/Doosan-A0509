;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ConfigDeleteTool)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ConfigDeleteTool (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::CONFIGDELETETOOL")
  (make-package "DSR_MSGS::CONFIGDELETETOOL"))
(unless (find-package "DSR_MSGS::CONFIGDELETETOOLREQUEST")
  (make-package "DSR_MSGS::CONFIGDELETETOOLREQUEST"))
(unless (find-package "DSR_MSGS::CONFIGDELETETOOLRESPONSE")
  (make-package "DSR_MSGS::CONFIGDELETETOOLRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::ConfigDeleteToolRequest
  :super ros::object
  :slots (_name ))

(defmethod dsr_msgs::ConfigDeleteToolRequest
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

(defclass dsr_msgs::ConfigDeleteToolResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::ConfigDeleteToolResponse
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

(defclass dsr_msgs::ConfigDeleteTool
  :super ros::object
  :slots ())

(setf (get dsr_msgs::ConfigDeleteTool :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::ConfigDeleteTool :datatype-) "dsr_msgs/ConfigDeleteTool")
(setf (get dsr_msgs::ConfigDeleteTool :request) dsr_msgs::ConfigDeleteToolRequest)
(setf (get dsr_msgs::ConfigDeleteTool :response) dsr_msgs::ConfigDeleteToolResponse)

(defmethod dsr_msgs::ConfigDeleteToolRequest
  (:response () (instance dsr_msgs::ConfigDeleteToolResponse :init)))

(setf (get dsr_msgs::ConfigDeleteToolRequest :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::ConfigDeleteToolRequest :datatype-) "dsr_msgs/ConfigDeleteToolRequest")
(setf (get dsr_msgs::ConfigDeleteToolRequest :definition-)
      "#____________________________________________________________________________________________
# config_delete_tool  
# It is a service to delete tool information registered in advance in the robot controller
#____________________________________________________________________________________________

string          name        # tool name 
---
bool success
")

(setf (get dsr_msgs::ConfigDeleteToolResponse :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::ConfigDeleteToolResponse :datatype-) "dsr_msgs/ConfigDeleteToolResponse")
(setf (get dsr_msgs::ConfigDeleteToolResponse :definition-)
      "#____________________________________________________________________________________________
# config_delete_tool  
# It is a service to delete tool information registered in advance in the robot controller
#____________________________________________________________________________________________

string          name        # tool name 
---
bool success
")



(provide :dsr_msgs/ConfigDeleteTool "d08a3b641c2f8680fbdfb1ea2e17a3e1")


