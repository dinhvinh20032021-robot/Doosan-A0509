;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetRTControlOutputVersionList)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetRTControlOutputVersionList (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETRTCONTROLOUTPUTVERSIONLIST")
  (make-package "DSR_MSGS::GETRTCONTROLOUTPUTVERSIONLIST"))
(unless (find-package "DSR_MSGS::GETRTCONTROLOUTPUTVERSIONLISTREQUEST")
  (make-package "DSR_MSGS::GETRTCONTROLOUTPUTVERSIONLISTREQUEST"))
(unless (find-package "DSR_MSGS::GETRTCONTROLOUTPUTVERSIONLISTRESPONSE")
  (make-package "DSR_MSGS::GETRTCONTROLOUTPUTVERSIONLISTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetRTControlOutputVersionListRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetRTControlOutputVersionListRequest
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

(defclass dsr_msgs::GetRTControlOutputVersionListResponse
  :super ros::object
  :slots (_success _version ))

(defmethod dsr_msgs::GetRTControlOutputVersionListResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:version __version) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _version (string __version))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:version
   (&optional __version)
   (if __version (setq _version __version)) _version)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _version
    4 (length _version)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _version
       (write-long (length _version) s) (princ _version s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _version
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _version (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass dsr_msgs::GetRTControlOutputVersionList
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetRTControlOutputVersionList :md5sum-) "621ab4fad3b2f7679edb9817178ccd57")
(setf (get dsr_msgs::GetRTControlOutputVersionList :datatype-) "dsr_msgs/GetRTControlOutputVersionList")
(setf (get dsr_msgs::GetRTControlOutputVersionList :request) dsr_msgs::GetRTControlOutputVersionListRequest)
(setf (get dsr_msgs::GetRTControlOutputVersionList :response) dsr_msgs::GetRTControlOutputVersionListResponse)

(defmethod dsr_msgs::GetRTControlOutputVersionListRequest
  (:response () (instance dsr_msgs::GetRTControlOutputVersionListResponse :init)))

(setf (get dsr_msgs::GetRTControlOutputVersionListRequest :md5sum-) "621ab4fad3b2f7679edb9817178ccd57")
(setf (get dsr_msgs::GetRTControlOutputVersionListRequest :datatype-) "dsr_msgs/GetRTControlOutputVersionListRequest")
(setf (get dsr_msgs::GetRTControlOutputVersionListRequest :definition-)
      "#____________________________________________________________________________________________
# get_rt_control_output_version_list
#____________________________________________________________________________________________

---
bool       success
string     version
")

(setf (get dsr_msgs::GetRTControlOutputVersionListResponse :md5sum-) "621ab4fad3b2f7679edb9817178ccd57")
(setf (get dsr_msgs::GetRTControlOutputVersionListResponse :datatype-) "dsr_msgs/GetRTControlOutputVersionListResponse")
(setf (get dsr_msgs::GetRTControlOutputVersionListResponse :definition-)
      "#____________________________________________________________________________________________
# get_rt_control_output_version_list
#____________________________________________________________________________________________

---
bool       success
string     version
")



(provide :dsr_msgs/GetRTControlOutputVersionList "621ab4fad3b2f7679edb9817178ccd57")


