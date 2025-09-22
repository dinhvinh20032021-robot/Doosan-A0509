;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetToolShape)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetToolShape (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETTOOLSHAPE")
  (make-package "DSR_MSGS::SETTOOLSHAPE"))
(unless (find-package "DSR_MSGS::SETTOOLSHAPEREQUEST")
  (make-package "DSR_MSGS::SETTOOLSHAPEREQUEST"))
(unless (find-package "DSR_MSGS::SETTOOLSHAPERESPONSE")
  (make-package "DSR_MSGS::SETTOOLSHAPERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetToolShapeRequest
  :super ros::object
  :slots (_name ))

(defmethod dsr_msgs::SetToolShapeRequest
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

(defclass dsr_msgs::SetToolShapeResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetToolShapeResponse
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

(defclass dsr_msgs::SetToolShape
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetToolShape :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::SetToolShape :datatype-) "dsr_msgs/SetToolShape")
(setf (get dsr_msgs::SetToolShape :request) dsr_msgs::SetToolShapeRequest)
(setf (get dsr_msgs::SetToolShape :response) dsr_msgs::SetToolShapeResponse)

(defmethod dsr_msgs::SetToolShapeRequest
  (:response () (instance dsr_msgs::SetToolShapeResponse :init)))

(setf (get dsr_msgs::SetToolShapeRequest :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::SetToolShapeRequest :datatype-) "dsr_msgs/SetToolShapeRequest")
(setf (get dsr_msgs::SetToolShapeRequest :definition-)
      "#____________________________________________________________________________________________
# set_tool_shape
# It is a service to set information about currently installed tool.  
# Activates the tool shape information of the entered name among the tool shape information registered in the Teach Pendant
#____________________________________________________________________________________________

string          name        # Tool name registered in the Teach Pendant
---
bool            success
")

(setf (get dsr_msgs::SetToolShapeResponse :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::SetToolShapeResponse :datatype-) "dsr_msgs/SetToolShapeResponse")
(setf (get dsr_msgs::SetToolShapeResponse :definition-)
      "#____________________________________________________________________________________________
# set_tool_shape
# It is a service to set information about currently installed tool.  
# Activates the tool shape information of the entered name among the tool shape information registered in the Teach Pendant
#____________________________________________________________________________________________

string          name        # Tool name registered in the Teach Pendant
---
bool            success
")



(provide :dsr_msgs/SetToolShape "d08a3b641c2f8680fbdfb1ea2e17a3e1")


