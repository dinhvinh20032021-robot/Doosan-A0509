;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ServoLStream)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ServoLStream (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SERVOLSTREAM")
  (make-package "DSR_MSGS::SERVOLSTREAM"))

(in-package "ROS")
;;//! \htmlinclude ServoLStream.msg.html


(defclass dsr_msgs::ServoLStream
  :super ros::object
  :slots (_pos _vel _acc _time ))

(defmethod dsr_msgs::ServoLStream
  (:init
   (&key
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:vel __vel) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:acc __acc) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:time __time) 0.0)
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _vel __vel)
   (setq _acc __acc)
   (setq _time (float __time))
   self)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
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
    ;; float64[6] _pos
    (* 8    6)
    ;; float64[2] _vel
    (* 8    2)
    ;; float64[2] _acc
    (* 8    2)
    ;; float64 _time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _pos
     (dotimes (i 6)
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[2] _vel
     (dotimes (i 2)
       (sys::poke (elt _vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[2] _acc
     (dotimes (i 2)
       (sys::poke (elt _acc i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[2] _vel
   (dotimes (i (length _vel))
     (setf (elt _vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[2] _acc
   (dotimes (i (length _acc))
     (setf (elt _acc i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get dsr_msgs::ServoLStream :md5sum-) "faef3261e269f88f638555affcbeea57")
(setf (get dsr_msgs::ServoLStream :datatype-) "dsr_msgs/ServoLStream")
(setf (get dsr_msgs::ServoLStream :definition-)
      "#____________________________________________________________________________________________
# servol
# 
#____________________________________________________________________________________________

float64[6] pos               # position  
float64[2] vel               # velocity
float64[2] acc               # acceleration
float64    time              # time
")



(provide :dsr_msgs/ServoLStream "faef3261e269f88f638555affcbeea57")


