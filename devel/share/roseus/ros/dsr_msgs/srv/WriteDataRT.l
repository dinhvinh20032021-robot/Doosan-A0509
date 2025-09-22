;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::WriteDataRT)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'WriteDataRT (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::WRITEDATART")
  (make-package "DSR_MSGS::WRITEDATART"))
(unless (find-package "DSR_MSGS::WRITEDATARTREQUEST")
  (make-package "DSR_MSGS::WRITEDATARTREQUEST"))
(unless (find-package "DSR_MSGS::WRITEDATARTRESPONSE")
  (make-package "DSR_MSGS::WRITEDATARTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::WriteDataRTRequest
  :super ros::object
  :slots (_external_force_torque _external_digital_input _external_digital_output _external_analog_input _external_analog_output ))

(defmethod dsr_msgs::WriteDataRTRequest
  (:init
   (&key
    ((:external_force_torque __external_force_torque) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:external_digital_input __external_digital_input) 0)
    ((:external_digital_output __external_digital_output) 0)
    ((:external_analog_input __external_analog_input) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:external_analog_output __external_analog_output) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _external_force_torque __external_force_torque)
   (setq _external_digital_input (round __external_digital_input))
   (setq _external_digital_output (round __external_digital_output))
   (setq _external_analog_input __external_analog_input)
   (setq _external_analog_output __external_analog_output)
   self)
  (:external_force_torque
   (&optional __external_force_torque)
   (if __external_force_torque (setq _external_force_torque __external_force_torque)) _external_force_torque)
  (:external_digital_input
   (&optional __external_digital_input)
   (if __external_digital_input (setq _external_digital_input __external_digital_input)) _external_digital_input)
  (:external_digital_output
   (&optional __external_digital_output)
   (if __external_digital_output (setq _external_digital_output __external_digital_output)) _external_digital_output)
  (:external_analog_input
   (&optional __external_analog_input)
   (if __external_analog_input (setq _external_analog_input __external_analog_input)) _external_analog_input)
  (:external_analog_output
   (&optional __external_analog_output)
   (if __external_analog_output (setq _external_analog_output __external_analog_output)) _external_analog_output)
  (:serialization-length
   ()
   (+
    ;; float64[6] _external_force_torque
    (* 8    6)
    ;; int32 _external_digital_input
    4
    ;; int32 _external_digital_output
    4
    ;; float64[6] _external_analog_input
    (* 8    6)
    ;; float64[6] _external_analog_output
    (* 8    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _external_force_torque
     (dotimes (i 6)
       (sys::poke (elt _external_force_torque i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32 _external_digital_input
       (write-long _external_digital_input s)
     ;; int32 _external_digital_output
       (write-long _external_digital_output s)
     ;; float64[6] _external_analog_input
     (dotimes (i 6)
       (sys::poke (elt _external_analog_input i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _external_analog_output
     (dotimes (i 6)
       (sys::poke (elt _external_analog_output i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _external_force_torque
   (dotimes (i (length _external_force_torque))
     (setf (elt _external_force_torque i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int32 _external_digital_input
     (setq _external_digital_input (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _external_digital_output
     (setq _external_digital_output (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[6] _external_analog_input
   (dotimes (i (length _external_analog_input))
     (setf (elt _external_analog_input i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _external_analog_output
   (dotimes (i (length _external_analog_output))
     (setf (elt _external_analog_output i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass dsr_msgs::WriteDataRTResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::WriteDataRTResponse
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

(defclass dsr_msgs::WriteDataRT
  :super ros::object
  :slots ())

(setf (get dsr_msgs::WriteDataRT :md5sum-) "57fbcf1b749a9e589d5aa77f87216c0e")
(setf (get dsr_msgs::WriteDataRT :datatype-) "dsr_msgs/WriteDataRT")
(setf (get dsr_msgs::WriteDataRT :request) dsr_msgs::WriteDataRTRequest)
(setf (get dsr_msgs::WriteDataRT :response) dsr_msgs::WriteDataRTResponse)

(defmethod dsr_msgs::WriteDataRTRequest
  (:response () (instance dsr_msgs::WriteDataRTResponse :init)))

(setf (get dsr_msgs::WriteDataRTRequest :md5sum-) "57fbcf1b749a9e589d5aa77f87216c0e")
(setf (get dsr_msgs::WriteDataRTRequest :datatype-) "dsr_msgs/WriteDataRTRequest")
(setf (get dsr_msgs::WriteDataRTRequest :definition-)
      "#____________________________________________________________________________________________
# write_data_rt
#____________________________________________________________________________________________
float64[6] external_force_torque
int32      external_digital_input
int32      external_digital_output
float64[6] external_analog_input
float64[6] external_analog_output
---
bool       success
")

(setf (get dsr_msgs::WriteDataRTResponse :md5sum-) "57fbcf1b749a9e589d5aa77f87216c0e")
(setf (get dsr_msgs::WriteDataRTResponse :datatype-) "dsr_msgs/WriteDataRTResponse")
(setf (get dsr_msgs::WriteDataRTResponse :definition-)
      "#____________________________________________________________________________________________
# write_data_rt
#____________________________________________________________________________________________
float64[6] external_force_torque
int32      external_digital_input
int32      external_digital_output
float64[6] external_analog_input
float64[6] external_analog_output
---
bool       success
")



(provide :dsr_msgs/WriteDataRT "57fbcf1b749a9e589d5aa77f87216c0e")


