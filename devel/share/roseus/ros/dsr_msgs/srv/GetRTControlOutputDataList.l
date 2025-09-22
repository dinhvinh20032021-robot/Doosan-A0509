;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetRTControlOutputDataList)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetRTControlOutputDataList (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETRTCONTROLOUTPUTDATALIST")
  (make-package "DSR_MSGS::GETRTCONTROLOUTPUTDATALIST"))
(unless (find-package "DSR_MSGS::GETRTCONTROLOUTPUTDATALISTREQUEST")
  (make-package "DSR_MSGS::GETRTCONTROLOUTPUTDATALISTREQUEST"))
(unless (find-package "DSR_MSGS::GETRTCONTROLOUTPUTDATALISTRESPONSE")
  (make-package "DSR_MSGS::GETRTCONTROLOUTPUTDATALISTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetRTControlOutputDataListRequest
  :super ros::object
  :slots (_version ))

(defmethod dsr_msgs::GetRTControlOutputDataListRequest
  (:init
   (&key
    ((:version __version) "")
    )
   (send-super :init)
   (setq _version (string __version))
   self)
  (:version
   (&optional __version)
   (if __version (setq _version __version)) _version)
  (:serialization-length
   ()
   (+
    ;; string _version
    4 (length _version)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _version
       (write-long (length _version) s) (princ _version s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _version
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _version (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass dsr_msgs::GetRTControlOutputDataListResponse
  :super ros::object
  :slots (_success _data ))

(defmethod dsr_msgs::GetRTControlOutputDataListResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:data __data) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _data (string __data))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _data
    4 (length _data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass dsr_msgs::GetRTControlOutputDataList
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetRTControlOutputDataList :md5sum-) "af5686a9e1b1a378bc0983d164821af1")
(setf (get dsr_msgs::GetRTControlOutputDataList :datatype-) "dsr_msgs/GetRTControlOutputDataList")
(setf (get dsr_msgs::GetRTControlOutputDataList :request) dsr_msgs::GetRTControlOutputDataListRequest)
(setf (get dsr_msgs::GetRTControlOutputDataList :response) dsr_msgs::GetRTControlOutputDataListResponse)

(defmethod dsr_msgs::GetRTControlOutputDataListRequest
  (:response () (instance dsr_msgs::GetRTControlOutputDataListResponse :init)))

(setf (get dsr_msgs::GetRTControlOutputDataListRequest :md5sum-) "af5686a9e1b1a378bc0983d164821af1")
(setf (get dsr_msgs::GetRTControlOutputDataListRequest :datatype-) "dsr_msgs/GetRTControlOutputDataListRequest")
(setf (get dsr_msgs::GetRTControlOutputDataListRequest :definition-)
      "#____________________________________________________________________________________________
# get_rt_control_output_data_list
#____________________________________________________________________________________________

string     version
---
bool       success
string     data
")

(setf (get dsr_msgs::GetRTControlOutputDataListResponse :md5sum-) "af5686a9e1b1a378bc0983d164821af1")
(setf (get dsr_msgs::GetRTControlOutputDataListResponse :datatype-) "dsr_msgs/GetRTControlOutputDataListResponse")
(setf (get dsr_msgs::GetRTControlOutputDataListResponse :definition-)
      "#____________________________________________________________________________________________
# get_rt_control_output_data_list
#____________________________________________________________________________________________

string     version
---
bool       success
string     data
")



(provide :dsr_msgs/GetRTControlOutputDataList "af5686a9e1b1a378bc0983d164821af1")


