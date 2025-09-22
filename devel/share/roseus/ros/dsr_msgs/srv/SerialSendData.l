;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SerialSendData)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SerialSendData (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SERIALSENDDATA")
  (make-package "DSR_MSGS::SERIALSENDDATA"))
(unless (find-package "DSR_MSGS::SERIALSENDDATAREQUEST")
  (make-package "DSR_MSGS::SERIALSENDDATAREQUEST"))
(unless (find-package "DSR_MSGS::SERIALSENDDATARESPONSE")
  (make-package "DSR_MSGS::SERIALSENDDATARESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SerialSendDataRequest
  :super ros::object
  :slots (_data ))

(defmethod dsr_msgs::SerialSendDataRequest
  (:init
   (&key
    ((:data __data) "")
    )
   (send-super :init)
   (setq _data (string __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string _data
    4 (length _data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass dsr_msgs::SerialSendDataResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SerialSendDataResponse
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

(defclass dsr_msgs::SerialSendData
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SerialSendData :md5sum-) "e7ac98ca304b04e45988c61adb6bc4ce")
(setf (get dsr_msgs::SerialSendData :datatype-) "dsr_msgs/SerialSendData")
(setf (get dsr_msgs::SerialSendData :request) dsr_msgs::SerialSendDataRequest)
(setf (get dsr_msgs::SerialSendData :response) dsr_msgs::SerialSendDataResponse)

(defmethod dsr_msgs::SerialSendDataRequest
  (:response () (instance dsr_msgs::SerialSendDataResponse :init)))

(setf (get dsr_msgs::SerialSendDataRequest :md5sum-) "e7ac98ca304b04e45988c61adb6bc4ce")
(setf (get dsr_msgs::SerialSendDataRequest :datatype-) "dsr_msgs/SerialSendDataRequest")
(setf (get dsr_msgs::SerialSendDataRequest :definition-)
      "#____________________________________________________________________________________________
# serial_send_data
# send byte data to other device 
# you can operate the real-robotiq gripper by modbus rtu
#____________________________________________________________________________________________

string data
---
bool success
")

(setf (get dsr_msgs::SerialSendDataResponse :md5sum-) "e7ac98ca304b04e45988c61adb6bc4ce")
(setf (get dsr_msgs::SerialSendDataResponse :datatype-) "dsr_msgs/SerialSendDataResponse")
(setf (get dsr_msgs::SerialSendDataResponse :definition-)
      "#____________________________________________________________________________________________
# serial_send_data
# send byte data to other device 
# you can operate the real-robotiq gripper by modbus rtu
#____________________________________________________________________________________________

string data
---
bool success
")



(provide :dsr_msgs/SerialSendData "e7ac98ca304b04e45988c61adb6bc4ce")


