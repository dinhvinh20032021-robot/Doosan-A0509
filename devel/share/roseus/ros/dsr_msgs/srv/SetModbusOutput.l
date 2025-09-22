;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetModbusOutput)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetModbusOutput (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETMODBUSOUTPUT")
  (make-package "DSR_MSGS::SETMODBUSOUTPUT"))
(unless (find-package "DSR_MSGS::SETMODBUSOUTPUTREQUEST")
  (make-package "DSR_MSGS::SETMODBUSOUTPUTREQUEST"))
(unless (find-package "DSR_MSGS::SETMODBUSOUTPUTRESPONSE")
  (make-package "DSR_MSGS::SETMODBUSOUTPUTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetModbusOutputRequest
  :super ros::object
  :slots (_name _value ))

(defmethod dsr_msgs::SetModbusOutputRequest
  (:init
   (&key
    ((:name __name) "")
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _value (round __value))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; int32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; int32 _value
       (write-long _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _value
     (setq _value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dsr_msgs::SetModbusOutputResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetModbusOutputResponse
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

(defclass dsr_msgs::SetModbusOutput
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetModbusOutput :md5sum-) "7e14dd01537ca042e8c020bcdd67b27a")
(setf (get dsr_msgs::SetModbusOutput :datatype-) "dsr_msgs/SetModbusOutput")
(setf (get dsr_msgs::SetModbusOutput :request) dsr_msgs::SetModbusOutputRequest)
(setf (get dsr_msgs::SetModbusOutput :response) dsr_msgs::SetModbusOutputResponse)

(defmethod dsr_msgs::SetModbusOutputRequest
  (:response () (instance dsr_msgs::SetModbusOutputResponse :init)))

(setf (get dsr_msgs::SetModbusOutputRequest :md5sum-) "7e14dd01537ca042e8c020bcdd67b27a")
(setf (get dsr_msgs::SetModbusOutputRequest :datatype-) "dsr_msgs/SetModbusOutputRequest")
(setf (get dsr_msgs::SetModbusOutputRequest :definition-)
      "#____________________________________________________________________________________________
# set_modbus_output  
# This service sends the signal to an external Modbus system. 
#____________________________________________________________________________________________

string      name     # modbus signal symbol
int32       value    # modbus register value
---
bool        success
")

(setf (get dsr_msgs::SetModbusOutputResponse :md5sum-) "7e14dd01537ca042e8c020bcdd67b27a")
(setf (get dsr_msgs::SetModbusOutputResponse :datatype-) "dsr_msgs/SetModbusOutputResponse")
(setf (get dsr_msgs::SetModbusOutputResponse :definition-)
      "#____________________________________________________________________________________________
# set_modbus_output  
# This service sends the signal to an external Modbus system. 
#____________________________________________________________________________________________

string      name     # modbus signal symbol
int32       value    # modbus register value
---
bool        success
")



(provide :dsr_msgs/SetModbusOutput "7e14dd01537ca042e8c020bcdd67b27a")


