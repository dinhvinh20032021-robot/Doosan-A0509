;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ConfigCreateModbus)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ConfigCreateModbus (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::CONFIGCREATEMODBUS")
  (make-package "DSR_MSGS::CONFIGCREATEMODBUS"))
(unless (find-package "DSR_MSGS::CONFIGCREATEMODBUSREQUEST")
  (make-package "DSR_MSGS::CONFIGCREATEMODBUSREQUEST"))
(unless (find-package "DSR_MSGS::CONFIGCREATEMODBUSRESPONSE")
  (make-package "DSR_MSGS::CONFIGCREATEMODBUSRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::ConfigCreateModbusRequest
  :super ros::object
  :slots (_name _ip _port _reg_type _index _value _slave_id ))

(defmethod dsr_msgs::ConfigCreateModbusRequest
  (:init
   (&key
    ((:name __name) "")
    ((:ip __ip) "")
    ((:port __port) 0)
    ((:reg_type __reg_type) 0)
    ((:index __index) 0)
    ((:value __value) 0)
    ((:slave_id __slave_id) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _ip (string __ip))
   (setq _port (round __port))
   (setq _reg_type (round __reg_type))
   (setq _index (round __index))
   (setq _value (round __value))
   (setq _slave_id (round __slave_id))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:ip
   (&optional __ip)
   (if __ip (setq _ip __ip)) _ip)
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:reg_type
   (&optional __reg_type)
   (if __reg_type (setq _reg_type __reg_type)) _reg_type)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:slave_id
   (&optional __slave_id)
   (if __slave_id (setq _slave_id __slave_id)) _slave_id)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _ip
    4 (length _ip)
    ;; int32 _port
    4
    ;; int8 _reg_type
    1
    ;; int8 _index
    1
    ;; int8 _value
    1
    ;; int32 _slave_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _ip
       (write-long (length _ip) s) (princ _ip s)
     ;; int32 _port
       (write-long _port s)
     ;; int8 _reg_type
       (write-byte _reg_type s)
     ;; int8 _index
       (write-byte _index s)
     ;; int8 _value
       (write-byte _value s)
     ;; int32 _slave_id
       (write-long _slave_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _ip
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ip (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _port
     (setq _port (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int8 _reg_type
     (setq _reg_type (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _reg_type 127) (setq _reg_type (- _reg_type 256)))
   ;; int8 _index
     (setq _index (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _index 127) (setq _index (- _index 256)))
   ;; int8 _value
     (setq _value (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _value 127) (setq _value (- _value 256)))
   ;; int32 _slave_id
     (setq _slave_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dsr_msgs::ConfigCreateModbusResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::ConfigCreateModbusResponse
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

(defclass dsr_msgs::ConfigCreateModbus
  :super ros::object
  :slots ())

(setf (get dsr_msgs::ConfigCreateModbus :md5sum-) "21b56fd5fb77f7fccfc8eb6d995cd64a")
(setf (get dsr_msgs::ConfigCreateModbus :datatype-) "dsr_msgs/ConfigCreateModbus")
(setf (get dsr_msgs::ConfigCreateModbus :request) dsr_msgs::ConfigCreateModbusRequest)
(setf (get dsr_msgs::ConfigCreateModbus :response) dsr_msgs::ConfigCreateModbusResponse)

(defmethod dsr_msgs::ConfigCreateModbusRequest
  (:response () (instance dsr_msgs::ConfigCreateModbusResponse :init)))

(setf (get dsr_msgs::ConfigCreateModbusRequest :md5sum-) "21b56fd5fb77f7fccfc8eb6d995cd64a")
(setf (get dsr_msgs::ConfigCreateModbusRequest :datatype-) "dsr_msgs/ConfigCreateModbusRequest")
(setf (get dsr_msgs::ConfigCreateModbusRequest :definition-)
      "#____________________________________________________________________________________________
# config_create_modbus  
# This service registers the Modbus signal. 
#____________________________________________________________________________________________

string      name       # modbus signal symbol 
string      ip         # external device ip
int32       port       # external device port     
int8        reg_type   # <MODBUS_REGISTER_TYPE>(0: discrete input, 1: coil, 2: input register, 3: holding register)
int8        index      # modbus signal index(0 ~ 9999)
int8        value      # modbus singla value(unsigned value ; 0 ~ 65535)
int32       slave_id   # Slave ID of the ModbusTCP(0: Broadcase address or 1-247 or 255: Default value for ModbusTCP) 
                       # <slave_id is only available in M2.40 or later versions>  
---
bool success
")

(setf (get dsr_msgs::ConfigCreateModbusResponse :md5sum-) "21b56fd5fb77f7fccfc8eb6d995cd64a")
(setf (get dsr_msgs::ConfigCreateModbusResponse :datatype-) "dsr_msgs/ConfigCreateModbusResponse")
(setf (get dsr_msgs::ConfigCreateModbusResponse :definition-)
      "#____________________________________________________________________________________________
# config_create_modbus  
# This service registers the Modbus signal. 
#____________________________________________________________________________________________

string      name       # modbus signal symbol 
string      ip         # external device ip
int32       port       # external device port     
int8        reg_type   # <MODBUS_REGISTER_TYPE>(0: discrete input, 1: coil, 2: input register, 3: holding register)
int8        index      # modbus signal index(0 ~ 9999)
int8        value      # modbus singla value(unsigned value ; 0 ~ 65535)
int32       slave_id   # Slave ID of the ModbusTCP(0: Broadcase address or 1-247 or 255: Default value for ModbusTCP) 
                       # <slave_id is only available in M2.40 or later versions>  
---
bool success
")



(provide :dsr_msgs/ConfigCreateModbus "21b56fd5fb77f7fccfc8eb6d995cd64a")


