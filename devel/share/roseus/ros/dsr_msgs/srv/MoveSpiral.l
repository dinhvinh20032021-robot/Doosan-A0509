;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::MoveSpiral)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'MoveSpiral (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::MOVESPIRAL")
  (make-package "DSR_MSGS::MOVESPIRAL"))
(unless (find-package "DSR_MSGS::MOVESPIRALREQUEST")
  (make-package "DSR_MSGS::MOVESPIRALREQUEST"))
(unless (find-package "DSR_MSGS::MOVESPIRALRESPONSE")
  (make-package "DSR_MSGS::MOVESPIRALRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::MoveSpiralRequest
  :super ros::object
  :slots (_revolution _maxRadius _maxLength _vel _acc _time _taskAxis _ref _syncType ))

(defmethod dsr_msgs::MoveSpiralRequest
  (:init
   (&key
    ((:revolution __revolution) 0.0)
    ((:maxRadius __maxRadius) 0.0)
    ((:maxLength __maxLength) 0.0)
    ((:vel __vel) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:acc __acc) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:time __time) 0.0)
    ((:taskAxis __taskAxis) 0)
    ((:ref __ref) 0)
    ((:syncType __syncType) 0)
    )
   (send-super :init)
   (setq _revolution (float __revolution))
   (setq _maxRadius (float __maxRadius))
   (setq _maxLength (float __maxLength))
   (setq _vel __vel)
   (setq _acc __acc)
   (setq _time (float __time))
   (setq _taskAxis (round __taskAxis))
   (setq _ref (round __ref))
   (setq _syncType (round __syncType))
   self)
  (:revolution
   (&optional __revolution)
   (if __revolution (setq _revolution __revolution)) _revolution)
  (:maxRadius
   (&optional __maxRadius)
   (if __maxRadius (setq _maxRadius __maxRadius)) _maxRadius)
  (:maxLength
   (&optional __maxLength)
   (if __maxLength (setq _maxLength __maxLength)) _maxLength)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:taskAxis
   (&optional __taskAxis)
   (if __taskAxis (setq _taskAxis __taskAxis)) _taskAxis)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:syncType
   (&optional __syncType)
   (if __syncType (setq _syncType __syncType)) _syncType)
  (:serialization-length
   ()
   (+
    ;; float64 _revolution
    8
    ;; float64 _maxRadius
    8
    ;; float64 _maxLength
    8
    ;; float64[2] _vel
    (* 8    2)
    ;; float64[2] _acc
    (* 8    2)
    ;; float64 _time
    8
    ;; int8 _taskAxis
    1
    ;; int8 _ref
    1
    ;; int8 _syncType
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _revolution
       (sys::poke _revolution (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _maxRadius
       (sys::poke _maxRadius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _maxLength
       (sys::poke _maxLength (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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
     ;; int8 _taskAxis
       (write-byte _taskAxis s)
     ;; int8 _ref
       (write-byte _ref s)
     ;; int8 _syncType
       (write-byte _syncType s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _revolution
     (setq _revolution (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _maxRadius
     (setq _maxRadius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _maxLength
     (setq _maxLength (sys::peek buf ptr- :double)) (incf ptr- 8)
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
   ;; int8 _taskAxis
     (setq _taskAxis (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _taskAxis 127) (setq _taskAxis (- _taskAxis 256)))
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;; int8 _syncType
     (setq _syncType (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _syncType 127) (setq _syncType (- _syncType 256)))
   ;;
   self)
  )

(defclass dsr_msgs::MoveSpiralResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::MoveSpiralResponse
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

(defclass dsr_msgs::MoveSpiral
  :super ros::object
  :slots ())

(setf (get dsr_msgs::MoveSpiral :md5sum-) "fc2c602cd43258b5eb06926d790138ea")
(setf (get dsr_msgs::MoveSpiral :datatype-) "dsr_msgs/MoveSpiral")
(setf (get dsr_msgs::MoveSpiral :request) dsr_msgs::MoveSpiralRequest)
(setf (get dsr_msgs::MoveSpiral :response) dsr_msgs::MoveSpiralResponse)

(defmethod dsr_msgs::MoveSpiralRequest
  (:response () (instance dsr_msgs::MoveSpiralResponse :init)))

(setf (get dsr_msgs::MoveSpiralRequest :md5sum-) "fc2c602cd43258b5eb06926d790138ea")
(setf (get dsr_msgs::MoveSpiralRequest :datatype-) "dsr_msgs/MoveSpiralRequest")
(setf (get dsr_msgs::MoveSpiralRequest :definition-)
      "#____________________________________________________________________________________________
# move_sprial  
#____________________________________________________________________________________________

float64    revolution       # Total number of revolutions 
float64    maxRadius        # Final spiral radius [mm]
float64    maxLength        # Distance moved in the axis direction [mm]
float64[2] vel              # set velocity: [mm/sec], [deg/sec]
float64[2] acc              # set acceleration: [mm/sec2], [deg/sec2]
float64    time #= 0.0      # Total execution time <sec> 
int8       taskAxis         # TASK_AXIS_X = 0, TASK_AXIS_Y = 1, TASK_AXIS_Z = 2   
int8       ref  #= 1        # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                            # <DR_WORLD is only available in M2.40 or later 
int8       syncType #=0     # SYNC = 0, ASYNC = 1 
---
bool success
")

(setf (get dsr_msgs::MoveSpiralResponse :md5sum-) "fc2c602cd43258b5eb06926d790138ea")
(setf (get dsr_msgs::MoveSpiralResponse :datatype-) "dsr_msgs/MoveSpiralResponse")
(setf (get dsr_msgs::MoveSpiralResponse :definition-)
      "#____________________________________________________________________________________________
# move_sprial  
#____________________________________________________________________________________________

float64    revolution       # Total number of revolutions 
float64    maxRadius        # Final spiral radius [mm]
float64    maxLength        # Distance moved in the axis direction [mm]
float64[2] vel              # set velocity: [mm/sec], [deg/sec]
float64[2] acc              # set acceleration: [mm/sec2], [deg/sec2]
float64    time #= 0.0      # Total execution time <sec> 
int8       taskAxis         # TASK_AXIS_X = 0, TASK_AXIS_Y = 1, TASK_AXIS_Z = 2   
int8       ref  #= 1        # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                            # <DR_WORLD is only available in M2.40 or later 
int8       syncType #=0     # SYNC = 0, ASYNC = 1 
---
bool success
")



(provide :dsr_msgs/MoveSpiral "fc2c602cd43258b5eb06926d790138ea")


