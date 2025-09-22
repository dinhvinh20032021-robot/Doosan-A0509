;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetRTControlInputVersionList)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetRTControlInputVersionList (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETRTCONTROLINPUTVERSIONLIST")
  (make-package "DSR_MSGS::GETRTCONTROLINPUTVERSIONLIST"))
(unless (find-package "DSR_MSGS::GETRTCONTROLINPUTVERSIONLISTREQUEST")
  (make-package "DSR_MSGS::GETRTCONTROLINPUTVERSIONLISTREQUEST"))
(unless (find-package "DSR_MSGS::GETRTCONTROLINPUTVERSIONLISTRESPONSE")
  (make-package "DSR_MSGS::GETRTCONTROLINPUTVERSIONLISTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetRTControlInputVersionListRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetRTControlInputVersionListRequest
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

(defclass dsr_msgs::GetRTControlInputVersionListResponse
  :super ros::object
  :slots (_success _version ))

(defmethod dsr_msgs::GetRTControlInputVersionListResponse
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

(defclass dsr_msgs::GetRTControlInputVersionList
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetRTControlInputVersionList :md5sum-) "621ab4fad3b2f7679edb9817178ccd57")
(setf (get dsr_msgs::GetRTControlInputVersionList :datatype-) "dsr_msgs/GetRTControlInputVersionList")
(setf (get dsr_msgs::GetRTControlInputVersionList :request) dsr_msgs::GetRTControlInputVersionListRequest)
(setf (get dsr_msgs::GetRTControlInputVersionList :response) dsr_msgs::GetRTControlInputVersionListResponse)

(defmethod dsr_msgs::GetRTControlInputVersionListRequest
  (:response () (instance dsr_msgs::GetRTControlInputVersionListResponse :init)))

(setf (get dsr_msgs::GetRTControlInputVersionListRequest :md5sum-) "621ab4fad3b2f7679edb9817178ccd57")
(setf (get dsr_msgs::GetRTControlInputVersionListRequest :datatype-) "dsr_msgs/GetRTControlInputVersionListRequest")
(setf (get dsr_msgs::GetRTControlInputVersionListRequest :definition-)
      "#____________________________________________________________________________________________
# get_rt_control_input_version_list
#____________________________________________________________________________________________

---
bool       success
string     version
")

(setf (get dsr_msgs::GetRTControlInputVersionListResponse :md5sum-) "621ab4fad3b2f7679edb9817178ccd57")
(setf (get dsr_msgs::GetRTControlInputVersionListResponse :datatype-) "dsr_msgs/GetRTControlInputVersionListResponse")
(setf (get dsr_msgs::GetRTControlInputVersionListResponse :definition-)
      "#____________________________________________________________________________________________
# get_rt_control_input_version_list
#____________________________________________________________________________________________

---
bool       success
string     version
")



(provide :dsr_msgs/GetRTControlInputVersionList "621ab4fad3b2f7679edb9817178ccd57")


