;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetCurrentTool)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetCurrentTool (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETCURRENTTOOL")
  (make-package "DSR_MSGS::GETCURRENTTOOL"))
(unless (find-package "DSR_MSGS::GETCURRENTTOOLREQUEST")
  (make-package "DSR_MSGS::GETCURRENTTOOLREQUEST"))
(unless (find-package "DSR_MSGS::GETCURRENTTOOLRESPONSE")
  (make-package "DSR_MSGS::GETCURRENTTOOLRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetCurrentToolRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetCurrentToolRequest
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

(defclass dsr_msgs::GetCurrentToolResponse
  :super ros::object
  :slots (_info _success ))

(defmethod dsr_msgs::GetCurrentToolResponse
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

(defclass dsr_msgs::GetCurrentTool
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetCurrentTool :md5sum-) "4ed9e7e2e2815d295b7f655584191cad")
(setf (get dsr_msgs::GetCurrentTool :datatype-) "dsr_msgs/GetCurrentTool")
(setf (get dsr_msgs::GetCurrentTool :request) dsr_msgs::GetCurrentToolRequest)
(setf (get dsr_msgs::GetCurrentTool :response) dsr_msgs::GetCurrentToolResponse)

(defmethod dsr_msgs::GetCurrentToolRequest
  (:response () (instance dsr_msgs::GetCurrentToolResponse :init)))

(setf (get dsr_msgs::GetCurrentToolRequest :md5sum-) "4ed9e7e2e2815d295b7f655584191cad")
(setf (get dsr_msgs::GetCurrentToolRequest :datatype-) "dsr_msgs/GetCurrentToolRequest")
(setf (get dsr_msgs::GetCurrentToolRequest :definition-)
      "#____________________________________________________________________________________________
# get_current_tool  
# It is a service to fetch the currently set tool information from the robot controller
#____________________________________________________________________________________________

---
string         info # tool name
bool        success
")

(setf (get dsr_msgs::GetCurrentToolResponse :md5sum-) "4ed9e7e2e2815d295b7f655584191cad")
(setf (get dsr_msgs::GetCurrentToolResponse :datatype-) "dsr_msgs/GetCurrentToolResponse")
(setf (get dsr_msgs::GetCurrentToolResponse :definition-)
      "#____________________________________________________________________________________________
# get_current_tool  
# It is a service to fetch the currently set tool information from the robot controller
#____________________________________________________________________________________________

---
string         info # tool name
bool        success
")



(provide :dsr_msgs/GetCurrentTool "4ed9e7e2e2815d295b7f655584191cad")


