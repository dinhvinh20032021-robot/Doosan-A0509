;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::TorqueRTStream)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'TorqueRTStream (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::TORQUERTSTREAM")
  (make-package "DSR_MSGS::TORQUERTSTREAM"))

(in-package "ROS")
;;//! \htmlinclude TorqueRTStream.msg.html


(defclass dsr_msgs::TorqueRTStream
  :super ros::object
  :slots (_tor _time ))

(defmethod dsr_msgs::TorqueRTStream
  (:init
   (&key
    ((:tor __tor) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:time __time) 0.0)
    )
   (send-super :init)
   (setq _tor __tor)
   (setq _time (float __time))
   self)
  (:tor
   (&optional __tor)
   (if __tor (setq _tor __tor)) _tor)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; float64[6] _tor
    (* 8    6)
    ;; float64 _time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _tor
     (dotimes (i 6)
       (sys::poke (elt _tor i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _tor
   (dotimes (i (length _tor))
     (setf (elt _tor i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get dsr_msgs::TorqueRTStream :md5sum-) "f87591a62093bf8e8f3ad8edb630f87d")
(setf (get dsr_msgs::TorqueRTStream :datatype-) "dsr_msgs/TorqueRTStream")
(setf (get dsr_msgs::TorqueRTStream :definition-)
      "#____________________________________________________________________________________________
# torque_rt
# 
#____________________________________________________________________________________________

float64[6] tor               # motor torque
float64    time              # time
")



(provide :dsr_msgs/TorqueRTStream "f87591a62093bf8e8f3ad8edb630f87d")


