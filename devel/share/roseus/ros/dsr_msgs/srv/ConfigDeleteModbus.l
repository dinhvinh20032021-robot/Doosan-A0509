;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ConfigDeleteModbus)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ConfigDeleteModbus (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::CONFIGDELETEMODBUS")
  (make-package "DSR_MSGS::CONFIGDELETEMODBUS"))
(unless (find-package "DSR_MSGS::CONFIGDELETEMODBUSREQUEST")
  (make-package "DSR_MSGS::CONFIGDELETEMODBUSREQUEST"))
(unless (find-package "DSR_MSGS::CONFIGDELETEMODBUSRESPONSE")
  (make-package "DSR_MSGS::CONFIGDELETEMODBUSRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::ConfigDeleteModbusRequest
  :super ros::object
  :slots (_name ))

(defmethod dsr_msgs::ConfigDeleteModbusRequest
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

(defclass dsr_msgs::ConfigDeleteModbusResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::ConfigDeleteModbusResponse
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

(defclass dsr_msgs::ConfigDeleteModbus
  :super ros::object
  :slots ())

(setf (get dsr_msgs::ConfigDeleteModbus :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::ConfigDeleteModbus :datatype-) "dsr_msgs/ConfigDeleteModbus")
(setf (get dsr_msgs::ConfigDeleteModbus :request) dsr_msgs::ConfigDeleteModbusRequest)
(setf (get dsr_msgs::ConfigDeleteModbus :response) dsr_msgs::ConfigDeleteModbusResponse)

(defmethod dsr_msgs::ConfigDeleteModbusRequest
  (:response () (instance dsr_msgs::ConfigDeleteModbusResponse :init)))

(setf (get dsr_msgs::ConfigDeleteModbusRequest :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::ConfigDeleteModbusRequest :datatype-) "dsr_msgs/ConfigDeleteModbusRequest")
(setf (get dsr_msgs::ConfigDeleteModbusRequest :definition-)
      "#____________________________________________________________________________________________
# config_delete_modbus 
# It is a service to delete the Modbus I / O signal information registered 
# in advance in the robot controller 
#____________________________________________________________________________________________

string      name       # modbus signal symbol 
---
bool success
")

(setf (get dsr_msgs::ConfigDeleteModbusResponse :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get dsr_msgs::ConfigDeleteModbusResponse :datatype-) "dsr_msgs/ConfigDeleteModbusResponse")
(setf (get dsr_msgs::ConfigDeleteModbusResponse :definition-)
      "#____________________________________________________________________________________________
# config_delete_modbus 
# It is a service to delete the Modbus I / O signal information registered 
# in advance in the robot controller 
#____________________________________________________________________________________________

string      name       # modbus signal symbol 
---
bool success
")



(provide :dsr_msgs/ConfigDeleteModbus "d08a3b641c2f8680fbdfb1ea2e17a3e1")


