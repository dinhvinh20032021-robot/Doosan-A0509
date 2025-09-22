;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::AlterMotionStream)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'AlterMotionStream (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::ALTERMOTIONSTREAM")
  (make-package "DSR_MSGS::ALTERMOTIONSTREAM"))

(in-package "ROS")
;;//! \htmlinclude AlterMotionStream.msg.html


(defclass dsr_msgs::AlterMotionStream
  :super ros::object
  :slots (_pos ))

(defmethod dsr_msgs::AlterMotionStream
  (:init
   (&key
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _pos __pos)
   self)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:serialization-length
   ()
   (+
    ;; float64[6] _pos
    (* 8    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _pos
     (dotimes (i 6)
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get dsr_msgs::AlterMotionStream :md5sum-) "537431324117c3d1847d70e057990155")
(setf (get dsr_msgs::AlterMotionStream :datatype-) "dsr_msgs/AlterMotionStream")
(setf (get dsr_msgs::AlterMotionStream :definition-)
      "#____________________________________________________________________________________________
# alter_motion  
# 
#____________________________________________________________________________________________

float64[6] pos               # position  

")



(provide :dsr_msgs/AlterMotionStream "537431324117c3d1847d70e057990155")


