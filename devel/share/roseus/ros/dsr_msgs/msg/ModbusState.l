;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ModbusState)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ModbusState (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::MODBUSSTATE")
  (make-package "DSR_MSGS::MODBUSSTATE"))

(in-package "ROS")
;;//! \htmlinclude ModbusState.msg.html


(defclass dsr_msgs::ModbusState
  :super ros::object
  :slots (_modbus_symbol _modbus_value ))

(defmethod dsr_msgs::ModbusState
  (:init
   (&key
    ((:modbus_symbol __modbus_symbol) "")
    ((:modbus_value __modbus_value) 0)
    )
   (send-super :init)
   (setq _modbus_symbol (string __modbus_symbol))
   (setq _modbus_value (round __modbus_value))
   self)
  (:modbus_symbol
   (&optional __modbus_symbol)
   (if __modbus_symbol (setq _modbus_symbol __modbus_symbol)) _modbus_symbol)
  (:modbus_value
   (&optional __modbus_value)
   (if __modbus_value (setq _modbus_value __modbus_value)) _modbus_value)
  (:serialization-length
   ()
   (+
    ;; string _modbus_symbol
    4 (length _modbus_symbol)
    ;; int32 _modbus_value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _modbus_symbol
       (write-long (length _modbus_symbol) s) (princ _modbus_symbol s)
     ;; int32 _modbus_value
       (write-long _modbus_value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _modbus_symbol
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _modbus_symbol (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _modbus_value
     (setq _modbus_value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dsr_msgs::ModbusState :md5sum-) "c941aa5cf7a235f21e3789f2d6e5305f")
(setf (get dsr_msgs::ModbusState :datatype-) "dsr_msgs/ModbusState")
(setf (get dsr_msgs::ModbusState :definition-)
      "#____________________________________________________________________________________________
#Custom msg for RobotState.msg -- MAX_SIZE = 100
#____________________________________________________________________________________________

string  modbus_symbol    # Modbus Signal Name
int32   modbus_value     # Modbus Register Value (Unsigned : 0 ~ 65535)
")



(provide :dsr_msgs/ModbusState "c941aa5cf7a235f21e3789f2d6e5305f")


