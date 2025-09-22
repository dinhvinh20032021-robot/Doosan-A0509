;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetModbusInput)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetModbusInput (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETMODBUSINPUT")
  (make-package "DSR_MSGS::GETMODBUSINPUT"))
(unless (find-package "DSR_MSGS::GETMODBUSINPUTREQUEST")
  (make-package "DSR_MSGS::GETMODBUSINPUTREQUEST"))
(unless (find-package "DSR_MSGS::GETMODBUSINPUTRESPONSE")
  (make-package "DSR_MSGS::GETMODBUSINPUTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetModbusInputRequest
  :super ros::object
  :slots (_name ))

(defmethod dsr_msgs::GetModbusInputRequest
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

(defclass dsr_msgs::GetModbusInputResponse
  :super ros::object
  :slots (_value _success ))

(defmethod dsr_msgs::GetModbusInputResponse
  (:init
   (&key
    ((:value __value) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _value (round __value))
   (setq _success __success)
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int32 _value
    4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _value
       (write-long _value s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _value
     (setq _value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetModbusInput
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetModbusInput :md5sum-) "a1db3dc69bfd7ece556367d29fe90139")
(setf (get dsr_msgs::GetModbusInput :datatype-) "dsr_msgs/GetModbusInput")
(setf (get dsr_msgs::GetModbusInput :request) dsr_msgs::GetModbusInputRequest)
(setf (get dsr_msgs::GetModbusInput :response) dsr_msgs::GetModbusInputResponse)

(defmethod dsr_msgs::GetModbusInputRequest
  (:response () (instance dsr_msgs::GetModbusInputResponse :init)))

(setf (get dsr_msgs::GetModbusInputRequest :md5sum-) "a1db3dc69bfd7ece556367d29fe90139")
(setf (get dsr_msgs::GetModbusInputRequest :datatype-) "dsr_msgs/GetModbusInputRequest")
(setf (get dsr_msgs::GetModbusInputRequest :definition-)
      "#____________________________________________________________________________________________
# get_modbus_input  
# This service reads the signal from the Modbus system.
#____________________________________________________________________________________________

string      name    # modbus signal symbol
---
int32       value    # modbus signal value
bool        success
")

(setf (get dsr_msgs::GetModbusInputResponse :md5sum-) "a1db3dc69bfd7ece556367d29fe90139")
(setf (get dsr_msgs::GetModbusInputResponse :datatype-) "dsr_msgs/GetModbusInputResponse")
(setf (get dsr_msgs::GetModbusInputResponse :definition-)
      "#____________________________________________________________________________________________
# get_modbus_input  
# This service reads the signal from the Modbus system.
#____________________________________________________________________________________________

string      name    # modbus signal symbol
---
int32       value    # modbus signal value
bool        success
")



(provide :dsr_msgs/GetModbusInput "a1db3dc69bfd7ece556367d29fe90139")


