;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SpeedJStream)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SpeedJStream (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SPEEDJSTREAM")
  (make-package "DSR_MSGS::SPEEDJSTREAM"))

(in-package "ROS")
;;//! \htmlinclude SpeedJStream.msg.html


(defclass dsr_msgs::SpeedJStream
  :super ros::object
  :slots (_vel _acc _time ))

(defmethod dsr_msgs::SpeedJStream
  (:init
   (&key
    ((:vel __vel) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:acc __acc) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:time __time) 0.0)
    )
   (send-super :init)
   (setq _vel __vel)
   (setq _acc __acc)
   (setq _time (float __time))
   self)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; float64[6] _vel
    (* 8    6)
    ;; float64[6] _acc
    (* 8    6)
    ;; float64 _time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _vel
     (dotimes (i 6)
       (sys::poke (elt _vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _acc
     (dotimes (i 6)
       (sys::poke (elt _acc i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _vel
   (dotimes (i (length _vel))
     (setf (elt _vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _acc
   (dotimes (i (length _acc))
     (setf (elt _acc i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get dsr_msgs::SpeedJStream :md5sum-) "e4ccbedcf19c8719d28eda24c562383b")
(setf (get dsr_msgs::SpeedJStream :datatype-) "dsr_msgs/SpeedJStream")
(setf (get dsr_msgs::SpeedJStream :definition-)
      "#____________________________________________________________________________________________
# speedj
# 
#____________________________________________________________________________________________

float64[6] vel               # velocity
float64[6] acc               # acceleration
float64    time              # time
")



(provide :dsr_msgs/SpeedJStream "e4ccbedcf19c8719d28eda24c562383b")


