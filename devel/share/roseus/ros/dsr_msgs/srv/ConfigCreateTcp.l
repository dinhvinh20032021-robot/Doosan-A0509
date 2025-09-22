;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ConfigCreateTcp)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ConfigCreateTcp (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::CONFIGCREATETCP")
  (make-package "DSR_MSGS::CONFIGCREATETCP"))
(unless (find-package "DSR_MSGS::CONFIGCREATETCPREQUEST")
  (make-package "DSR_MSGS::CONFIGCREATETCPREQUEST"))
(unless (find-package "DSR_MSGS::CONFIGCREATETCPRESPONSE")
  (make-package "DSR_MSGS::CONFIGCREATETCPRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::ConfigCreateTcpRequest
  :super ros::object
  :slots (_name _pos ))

(defmethod dsr_msgs::ConfigCreateTcpRequest
  (:init
   (&key
    ((:name __name) "")
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _pos __pos)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float64[6] _pos
    (* 8    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float64[6] _pos
     (dotimes (i 6)
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64[6] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass dsr_msgs::ConfigCreateTcpResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::ConfigCreateTcpResponse
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

(defclass dsr_msgs::ConfigCreateTcp
  :super ros::object
  :slots ())

(setf (get dsr_msgs::ConfigCreateTcp :md5sum-) "0c74c9733ff924f346cde676e47aa534")
(setf (get dsr_msgs::ConfigCreateTcp :datatype-) "dsr_msgs/ConfigCreateTcp")
(setf (get dsr_msgs::ConfigCreateTcp :request) dsr_msgs::ConfigCreateTcpRequest)
(setf (get dsr_msgs::ConfigCreateTcp :response) dsr_msgs::ConfigCreateTcpResponse)

(defmethod dsr_msgs::ConfigCreateTcpRequest
  (:response () (instance dsr_msgs::ConfigCreateTcpResponse :init)))

(setf (get dsr_msgs::ConfigCreateTcpRequest :md5sum-) "0c74c9733ff924f346cde676e47aa534")
(setf (get dsr_msgs::ConfigCreateTcpRequest :datatype-) "dsr_msgs/ConfigCreateTcpRequest")
(setf (get dsr_msgs::ConfigCreateTcpRequest :definition-)
      "#____________________________________________________________________________________________
# config_create_tcp  
# It is a service for registering and using robot TCP information in advance for safety
#____________________________________________________________________________________________

string          name         # tcp name 
float64[6]      pos          # coordinates of the TCP 
---
bool success
")

(setf (get dsr_msgs::ConfigCreateTcpResponse :md5sum-) "0c74c9733ff924f346cde676e47aa534")
(setf (get dsr_msgs::ConfigCreateTcpResponse :datatype-) "dsr_msgs/ConfigCreateTcpResponse")
(setf (get dsr_msgs::ConfigCreateTcpResponse :definition-)
      "#____________________________________________________________________________________________
# config_create_tcp  
# It is a service for registering and using robot TCP information in advance for safety
#____________________________________________________________________________________________

string          name         # tcp name 
float64[6]      pos          # coordinates of the TCP 
---
bool success
")



(provide :dsr_msgs/ConfigCreateTcp "0c74c9733ff924f346cde676e47aa534")


