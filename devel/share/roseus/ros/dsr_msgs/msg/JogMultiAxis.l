;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::JogMultiAxis)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'JogMultiAxis (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::JOGMULTIAXIS")
  (make-package "DSR_MSGS::JOGMULTIAXIS"))

(in-package "ROS")
;;//! \htmlinclude JogMultiAxis.msg.html


(defclass dsr_msgs::JogMultiAxis
  :super ros::object
  :slots (_jog_axis _move_reference _speed ))

(defmethod dsr_msgs::JogMultiAxis
  (:init
   (&key
    ((:jog_axis __jog_axis) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:move_reference __move_reference) 0)
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _jog_axis __jog_axis)
   (setq _move_reference (round __move_reference))
   (setq _speed (float __speed))
   self)
  (:jog_axis
   (&optional __jog_axis)
   (if __jog_axis (setq _jog_axis __jog_axis)) _jog_axis)
  (:move_reference
   (&optional __move_reference)
   (if __move_reference (setq _move_reference __move_reference)) _move_reference)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; float64[6] _jog_axis
    (* 8    6)
    ;; int8 _move_reference
    1
    ;; float64 _speed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _jog_axis
     (dotimes (i 6)
       (sys::poke (elt _jog_axis i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8 _move_reference
       (write-byte _move_reference s)
     ;; float64 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _jog_axis
   (dotimes (i (length _jog_axis))
     (setf (elt _jog_axis i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int8 _move_reference
     (setq _move_reference (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _move_reference 127) (setq _move_reference (- _move_reference 256)))
   ;; float64 _speed
     (setq _speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get dsr_msgs::JogMultiAxis :md5sum-) "e36e615ff2ffad91791b721818ab6be9")
(setf (get dsr_msgs::JogMultiAxis :datatype-) "dsr_msgs/JogMultiAxis")
(setf (get dsr_msgs::JogMultiAxis :definition-)
      "#____________________________________________________________________________________________
# multi jog
# multi jog speed = (250mm/s x 1.73) x unit vecter x speed [%] 
#____________________________________________________________________________________________

float64[6]  jog_axis          # unit vecter of Task space [Tx, Ty, Tz, Rx, Ry, Rz] : -1.0 ~ +1.0 
int8        move_reference    # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL, 2 : MOVE_REFERENCE_WORLD
float64     speed             # jog speed [%]
")



(provide :dsr_msgs/JogMultiAxis "e36e615ff2ffad91791b721818ab6be9")


