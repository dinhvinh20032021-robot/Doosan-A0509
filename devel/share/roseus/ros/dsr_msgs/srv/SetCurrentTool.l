;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetCurrentTool)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetCurrentTool (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETCURRENTTOOL")
  (make-package "DSR_MSGS::SETCURRENTTOOL"))
(unless (find-package "DSR_MSGS::SETCURRENTTOOLREQUEST")
  (make-package "DSR_MSGS::SETCURRENTTOOLREQUEST"))
(unless (find-package "DSR_MSGS::SETCURRENTTOOLRESPONSE")
  (make-package "DSR_MSGS::SETCURRENTTOOLRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetCurrentToolRequest
  :super ros::object
  :slots (_name ))

(defmethod dsr_msgs::SetCurrentToolRequest
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

(defclass dsr_msgs::SetCurrentToolResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetCurrentToolResponse
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

(defclass dsr_msgs::SetCurrentTool
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetCurrentTool :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::SetCurrentTool :datatype-) "dsr_msgs/SetCurrentTool")
(setf (get dsr_msgs::SetCurrentTool :request) dsr_msgs::SetCurrentToolRequest)
(setf (get dsr_msgs::SetCurrentTool :response) dsr_msgs::SetCurrentToolResponse)

(defmethod dsr_msgs::SetCurrentToolRequest
  (:response () (instance dsr_msgs::SetCurrentToolResponse :init)))

(setf (get dsr_msgs::SetCurrentToolRequest :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::SetCurrentToolRequest :datatype-) "dsr_msgs/SetCurrentToolRequest")
(setf (get dsr_msgs::SetCurrentToolRequest :definition-)
      "#____________________________________________________________________________________________
# set_current_tool
# It is a service to set information about currently installed tool.  
#____________________________________________________________________________________________

string          name        # tool name
---
bool            success
")

(setf (get dsr_msgs::SetCurrentToolResponse :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::SetCurrentToolResponse :datatype-) "dsr_msgs/SetCurrentToolResponse")
(setf (get dsr_msgs::SetCurrentToolResponse :definition-)
      "#____________________________________________________________________________________________
# set_current_tool
# It is a service to set information about currently installed tool.  
#____________________________________________________________________________________________

string          name        # tool name
---
bool            success
")



(provide :dsr_msgs/SetCurrentTool "d08a3b641c2f8680fbdfb1ea2e17a3e1")


