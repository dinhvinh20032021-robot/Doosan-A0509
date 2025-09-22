;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::CheckPositionCondition)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'CheckPositionCondition (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::CHECKPOSITIONCONDITION")
  (make-package "DSR_MSGS::CHECKPOSITIONCONDITION"))
(unless (find-package "DSR_MSGS::CHECKPOSITIONCONDITIONREQUEST")
  (make-package "DSR_MSGS::CHECKPOSITIONCONDITIONREQUEST"))
(unless (find-package "DSR_MSGS::CHECKPOSITIONCONDITIONRESPONSE")
  (make-package "DSR_MSGS::CHECKPOSITIONCONDITIONRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::CheckPositionConditionRequest
  :super ros::object
  :slots (_axis _min _max _ref _mode _pos ))

(defmethod dsr_msgs::CheckPositionConditionRequest
  (:init
   (&key
    ((:axis __axis) 0)
    ((:min __min) 0.0)
    ((:max __max) 0.0)
    ((:ref __ref) 0)
    ((:mode __mode) 0)
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _axis (round __axis))
   (setq _min (float __min))
   (setq _max (float __max))
   (setq _ref (round __ref))
   (setq _mode (round __mode))
   (setq _pos __pos)
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
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:serialization-length
   ()
   (+
    ;; int8 _axis
    1
    ;; float64 _min
    8
    ;; float64 _max
    8
    ;; int8 _ref
    1
    ;; int8 _mode
    1
    ;; float64[6] _pos
    (* 8    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _axis
       (write-byte _axis s)
     ;; float64 _min
       (sys::poke _min (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max
       (sys::poke _max (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8 _ref
       (write-byte _ref s)
     ;; int8 _mode
       (write-byte _mode s)
     ;; float64[6] _pos
     (dotimes (i 6)
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _axis
     (setq _axis (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _axis 127) (setq _axis (- _axis 256)))
   ;; float64 _min
     (setq _min (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max
     (setq _max (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;; int8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;; float64[6] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass dsr_msgs::CheckPositionConditionResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::CheckPositionConditionResponse
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

(defclass dsr_msgs::CheckPositionCondition
  :super ros::object
  :slots ())

(setf (get dsr_msgs::CheckPositionCondition :md5sum-) "d57ced9cfdce36e99486179c4a55944c")
(setf (get dsr_msgs::CheckPositionCondition :datatype-) "dsr_msgs/CheckPositionCondition")
(setf (get dsr_msgs::CheckPositionCondition :request) dsr_msgs::CheckPositionConditionRequest)
(setf (get dsr_msgs::CheckPositionCondition :response) dsr_msgs::CheckPositionConditionResponse)

(defmethod dsr_msgs::CheckPositionConditionRequest
  (:response () (instance dsr_msgs::CheckPositionConditionResponse :init)))

(setf (get dsr_msgs::CheckPositionConditionRequest :md5sum-) "d57ced9cfdce36e99486179c4a55944c")
(setf (get dsr_msgs::CheckPositionConditionRequest :datatype-) "dsr_msgs/CheckPositionConditionRequest")
(setf (get dsr_msgs::CheckPositionConditionRequest :definition-)
      "#____________________________________________________________________________________________
# check_position_condition  
#____________________________________________________________________________________________

int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
float64    min               # min    
float64    max               # max  
int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user_coordinate(101~200)
                             # <DR_WORLD is only available in M2.40 or later> 
int8       mode #= 0         # DR_MV_MOD_ABS(0), DR_MV_MOD_REL(1) 
float64[6] pos               # task pos(posx)  
---
bool success                 # True or False
")

(setf (get dsr_msgs::CheckPositionConditionResponse :md5sum-) "d57ced9cfdce36e99486179c4a55944c")
(setf (get dsr_msgs::CheckPositionConditionResponse :datatype-) "dsr_msgs/CheckPositionConditionResponse")
(setf (get dsr_msgs::CheckPositionConditionResponse :definition-)
      "#____________________________________________________________________________________________
# check_position_condition  
#____________________________________________________________________________________________

int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
float64    min               # min    
float64    max               # max  
int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user_coordinate(101~200)
                             # <DR_WORLD is only available in M2.40 or later> 
int8       mode #= 0         # DR_MV_MOD_ABS(0), DR_MV_MOD_REL(1) 
float64[6] pos               # task pos(posx)  
---
bool success                 # True or False
")



(provide :dsr_msgs/CheckPositionCondition "d57ced9cfdce36e99486179c4a55944c")


