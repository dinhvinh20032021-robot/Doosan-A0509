;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::CheckOrientationCondition1)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'CheckOrientationCondition1 (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::CHECKORIENTATIONCONDITION1")
  (make-package "DSR_MSGS::CHECKORIENTATIONCONDITION1"))
(unless (find-package "DSR_MSGS::CHECKORIENTATIONCONDITION1REQUEST")
  (make-package "DSR_MSGS::CHECKORIENTATIONCONDITION1REQUEST"))
(unless (find-package "DSR_MSGS::CHECKORIENTATIONCONDITION1RESPONSE")
  (make-package "DSR_MSGS::CHECKORIENTATIONCONDITION1RESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::CheckOrientationCondition1Request
  :super ros::object
  :slots (_axis _min _max _ref _mode ))

(defmethod dsr_msgs::CheckOrientationCondition1Request
  (:init
   (&key
    ((:axis __axis) 0)
    ((:min __min) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:max __max) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:ref __ref) 0)
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _axis (round __axis))
   (setq _min __min)
   (setq _max __max)
   (setq _ref (round __ref))
   (setq _mode (round __mode))
   self)
  (:axis
   (&optional __axis)
   (if __axis (setq _axis __axis)) _axis)
  (:min
   (&optional __min)
   (if __min (setq _min __min)) _min)
  (:max
   (&optional __max)
   (if __max (setq _max __max)) _max)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; int8 _axis
    1
    ;; float64[6] _min
    (* 8    6)
    ;; float64[6] _max
    (* 8    6)
    ;; int8 _ref
    1
    ;; int8 _mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _axis
       (write-byte _axis s)
     ;; float64[6] _min
     (dotimes (i 6)
       (sys::poke (elt _min i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _max
     (dotimes (i 6)
       (sys::poke (elt _max i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8 _ref
       (write-byte _ref s)
     ;; int8 _mode
       (write-byte _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _axis
     (setq _axis (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _axis 127) (setq _axis (- _axis 256)))
   ;; float64[6] _min
   (dotimes (i (length _min))
     (setf (elt _min i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _max
   (dotimes (i (length _max))
     (setf (elt _max i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;; int8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;;
   self)
  )

(defclass dsr_msgs::CheckOrientationCondition1Response
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::CheckOrientationCondition1Response
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

(defclass dsr_msgs::CheckOrientationCondition1
  :super ros::object
  :slots ())

(setf (get dsr_msgs::CheckOrientationCondition1 :md5sum-) "fbde9efadab849c82edf01bc49976a6a")
(setf (get dsr_msgs::CheckOrientationCondition1 :datatype-) "dsr_msgs/CheckOrientationCondition1")
(setf (get dsr_msgs::CheckOrientationCondition1 :request) dsr_msgs::CheckOrientationCondition1Request)
(setf (get dsr_msgs::CheckOrientationCondition1 :response) dsr_msgs::CheckOrientationCondition1Response)

(defmethod dsr_msgs::CheckOrientationCondition1Request
  (:response () (instance dsr_msgs::CheckOrientationCondition1Response :init)))

(setf (get dsr_msgs::CheckOrientationCondition1Request :md5sum-) "fbde9efadab849c82edf01bc49976a6a")
(setf (get dsr_msgs::CheckOrientationCondition1Request :datatype-) "dsr_msgs/CheckOrientationCondition1Request")
(setf (get dsr_msgs::CheckOrientationCondition1Request :definition-)
      "#____________________________________________________________________________________________
# check_orientation_condition(axis, min, max, ref, mod)  
#____________________________________________________________________________________________

int8       axis              # DR_AXIS_A(10), DR_AXIS_B(11), DR_AXIS_C(12) 
float64[6] min               # task pos(posx)  
float64[6] max               # task pos(posx)  
int8       ref  #= 0         # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user_coordinate(101~200)
                             # <DR_WORLD is only available in M2.40 or later> 
int8       mode #= 0         # DR_MV_MOD_ABS(0)
---
bool success                 # True or False
")

(setf (get dsr_msgs::CheckOrientationCondition1Response :md5sum-) "fbde9efadab849c82edf01bc49976a6a")
(setf (get dsr_msgs::CheckOrientationCondition1Response :datatype-) "dsr_msgs/CheckOrientationCondition1Response")
(setf (get dsr_msgs::CheckOrientationCondition1Response :definition-)
      "#____________________________________________________________________________________________
# check_orientation_condition(axis, min, max, ref, mod)  
#____________________________________________________________________________________________

int8       axis              # DR_AXIS_A(10), DR_AXIS_B(11), DR_AXIS_C(12) 
float64[6] min               # task pos(posx)  
float64[6] max               # task pos(posx)  
int8       ref  #= 0         # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user_coordinate(101~200)
                             # <DR_WORLD is only available in M2.40 or later> 
int8       mode #= 0         # DR_MV_MOD_ABS(0)
---
bool success                 # True or False
")



(provide :dsr_msgs/CheckOrientationCondition1 "fbde9efadab849c82edf01bc49976a6a")


