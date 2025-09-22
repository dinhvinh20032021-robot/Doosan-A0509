;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::AlterMotion)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'AlterMotion (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::ALTERMOTION")
  (make-package "DSR_MSGS::ALTERMOTION"))
(unless (find-package "DSR_MSGS::ALTERMOTIONREQUEST")
  (make-package "DSR_MSGS::ALTERMOTIONREQUEST"))
(unless (find-package "DSR_MSGS::ALTERMOTIONRESPONSE")
  (make-package "DSR_MSGS::ALTERMOTIONRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::AlterMotionRequest
  :super ros::object
  :slots (_pos ))

(defmethod dsr_msgs::AlterMotionRequest
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

(defclass dsr_msgs::AlterMotionResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::AlterMotionResponse
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

(defclass dsr_msgs::AlterMotion
  :super ros::object
  :slots ())

(setf (get dsr_msgs::AlterMotion :md5sum-) "d61037f3a31077a87fa540cd63743057")
(setf (get dsr_msgs::AlterMotion :datatype-) "dsr_msgs/AlterMotion")
(setf (get dsr_msgs::AlterMotion :request) dsr_msgs::AlterMotionRequest)
(setf (get dsr_msgs::AlterMotion :response) dsr_msgs::AlterMotionResponse)

(defmethod dsr_msgs::AlterMotionRequest
  (:response () (instance dsr_msgs::AlterMotionResponse :init)))

(setf (get dsr_msgs::AlterMotionRequest :md5sum-) "d61037f3a31077a87fa540cd63743057")
(setf (get dsr_msgs::AlterMotionRequest :datatype-) "dsr_msgs/AlterMotionRequest")
(setf (get dsr_msgs::AlterMotionRequest :definition-)
      "#____________________________________________________________________________________________
# alter_motion  
# 
#____________________________________________________________________________________________

float64[6] pos               # position  
---
bool success
")

(setf (get dsr_msgs::AlterMotionResponse :md5sum-) "d61037f3a31077a87fa540cd63743057")
(setf (get dsr_msgs::AlterMotionResponse :datatype-) "dsr_msgs/AlterMotionResponse")
(setf (get dsr_msgs::AlterMotionResponse :definition-)
      "#____________________________________________________________________________________________
# alter_motion  
# 
#____________________________________________________________________________________________

float64[6] pos               # position  
---
bool success
")



(provide :dsr_msgs/AlterMotion "d61037f3a31077a87fa540cd63743057")


