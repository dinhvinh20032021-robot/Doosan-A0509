;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::EnableAlterMotion)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'EnableAlterMotion (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::ENABLEALTERMOTION")
  (make-package "DSR_MSGS::ENABLEALTERMOTION"))
(unless (find-package "DSR_MSGS::ENABLEALTERMOTIONREQUEST")
  (make-package "DSR_MSGS::ENABLEALTERMOTIONREQUEST"))
(unless (find-package "DSR_MSGS::ENABLEALTERMOTIONRESPONSE")
  (make-package "DSR_MSGS::ENABLEALTERMOTIONRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::EnableAlterMotionRequest
  :super ros::object
  :slots (_n _mode _ref _limit_dPOS _limit_dPOS_per ))

(defmethod dsr_msgs::EnableAlterMotionRequest
  (:init
   (&key
    ((:n __n) 0)
    ((:mode __mode) 0)
    ((:ref __ref) 0)
    ((:limit_dPOS __limit_dPOS) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:limit_dPOS_per __limit_dPOS_per) (make-array 2 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _n (round __n))
   (setq _mode (round __mode))
   (setq _ref (round __ref))
   (setq _limit_dPOS __limit_dPOS)
   (setq _limit_dPOS_per __limit_dPOS_per)
   self)
  (:n
   (&optional __n)
   (if __n (setq _n __n)) _n)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:limit_dPOS
   (&optional __limit_dPOS)
   (if __limit_dPOS (setq _limit_dPOS __limit_dPOS)) _limit_dPOS)
  (:limit_dPOS_per
   (&optional __limit_dPOS_per)
   (if __limit_dPOS_per (setq _limit_dPOS_per __limit_dPOS_per)) _limit_dPOS_per)
  (:serialization-length
   ()
   (+
    ;; int32 _n
    4
    ;; int8 _mode
    1
    ;; int8 _ref
    1
    ;; float64[2] _limit_dPOS
    (* 8    2)
    ;; float64[2] _limit_dPOS_per
    (* 8    2)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _n
       (write-long _n s)
     ;; int8 _mode
       (write-byte _mode s)
     ;; int8 _ref
       (write-byte _ref s)
     ;; float64[2] _limit_dPOS
     (dotimes (i 2)
       (sys::poke (elt _limit_dPOS i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[2] _limit_dPOS_per
     (dotimes (i 2)
       (sys::poke (elt _limit_dPOS_per i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _n
     (setq _n (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;; float64[2] _limit_dPOS
   (dotimes (i (length _limit_dPOS))
     (setf (elt _limit_dPOS i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[2] _limit_dPOS_per
   (dotimes (i (length _limit_dPOS_per))
     (setf (elt _limit_dPOS_per i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass dsr_msgs::EnableAlterMotionResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::EnableAlterMotionResponse
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

(defclass dsr_msgs::EnableAlterMotion
  :super ros::object
  :slots ())

(setf (get dsr_msgs::EnableAlterMotion :md5sum-) "4bc090a8b26dd2353ca78fe889287b7f")
(setf (get dsr_msgs::EnableAlterMotion :datatype-) "dsr_msgs/EnableAlterMotion")
(setf (get dsr_msgs::EnableAlterMotion :request) dsr_msgs::EnableAlterMotionRequest)
(setf (get dsr_msgs::EnableAlterMotion :response) dsr_msgs::EnableAlterMotionResponse)

(defmethod dsr_msgs::EnableAlterMotionRequest
  (:response () (instance dsr_msgs::EnableAlterMotionResponse :init)))

(setf (get dsr_msgs::EnableAlterMotionRequest :md5sum-) "4bc090a8b26dd2353ca78fe889287b7f")
(setf (get dsr_msgs::EnableAlterMotionRequest :datatype-) "dsr_msgs/EnableAlterMotionRequest")
(setf (get dsr_msgs::EnableAlterMotionRequest :definition-)
      "#____________________________________________________________________________________________
# enable_alter_motion  
# 
#____________________________________________________________________________________________

int32      n                 # Cycle time number 
int8       mode              # DR_DPOS(0) : accumulation amount, DR_DVEL(1) : increment amount 
int8       ref               # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200) 
                             # <ref is only available in M2.40 or later> 
float64[2] limit_dPOS        # First value : limitation of position[mm], Second value : limitation of orientation[deg]
float64[2] limit_dPOS_per    # First value : limitation of position[mm], Second value : limitation of orientation[deg]

---
bool success
")

(setf (get dsr_msgs::EnableAlterMotionResponse :md5sum-) "4bc090a8b26dd2353ca78fe889287b7f")
(setf (get dsr_msgs::EnableAlterMotionResponse :datatype-) "dsr_msgs/EnableAlterMotionResponse")
(setf (get dsr_msgs::EnableAlterMotionResponse :definition-)
      "#____________________________________________________________________________________________
# enable_alter_motion  
# 
#____________________________________________________________________________________________

int32      n                 # Cycle time number 
int8       mode              # DR_DPOS(0) : accumulation amount, DR_DVEL(1) : increment amount 
int8       ref               # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200) 
                             # <ref is only available in M2.40 or later> 
float64[2] limit_dPOS        # First value : limitation of position[mm], Second value : limitation of orientation[deg]
float64[2] limit_dPOS_per    # First value : limitation of position[mm], Second value : limitation of orientation[deg]

---
bool success
")



(provide :dsr_msgs/EnableAlterMotion "4bc090a8b26dd2353ca78fe889287b7f")


