;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::MoveJointx)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'MoveJointx (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::MOVEJOINTX")
  (make-package "DSR_MSGS::MOVEJOINTX"))
(unless (find-package "DSR_MSGS::MOVEJOINTXREQUEST")
  (make-package "DSR_MSGS::MOVEJOINTXREQUEST"))
(unless (find-package "DSR_MSGS::MOVEJOINTXRESPONSE")
  (make-package "DSR_MSGS::MOVEJOINTXRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::MoveJointxRequest
  :super ros::object
  :slots (_pos _vel _acc _time _radius _ref _mode _blendType _sol _syncType ))

(defmethod dsr_msgs::MoveJointxRequest
  (:init
   (&key
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:vel __vel) 0.0)
    ((:acc __acc) 0.0)
    ((:time __time) 0.0)
    ((:radius __radius) 0.0)
    ((:ref __ref) 0)
    ((:mode __mode) 0)
    ((:blendType __blendType) 0)
    ((:sol __sol) 0)
    ((:syncType __syncType) 0)
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _vel (float __vel))
   (setq _acc (float __acc))
   (setq _time (float __time))
   (setq _radius (float __radius))
   (setq _ref (round __ref))
   (setq _mode (round __mode))
   (setq _blendType (round __blendType))
   (setq _sol (round __sol))
   (setq _syncType (round __syncType))
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
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:blendType
   (&optional __blendType)
   (if __blendType (setq _blendType __blendType)) _blendType)
  (:sol
   (&optional __sol)
   (if __sol (setq _sol __sol)) _sol)
  (:syncType
   (&optional __syncType)
   (if __syncType (setq _syncType __syncType)) _syncType)
  (:serialization-length
   ()
   (+
    ;; float64[6] _pos
    (* 8    6)
    ;; float64 _vel
    8
    ;; float64 _acc
    8
    ;; float64 _time
    8
    ;; float64 _radius
    8
    ;; int8 _ref
    1
    ;; int8 _mode
    1
    ;; int8 _blendType
    1
    ;; int8 _sol
    1
    ;; int8 _syncType
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _pos
     (dotimes (i 6)
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _vel
       (sys::poke _vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _acc
       (sys::poke _acc (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8 _ref
       (write-byte _ref s)
     ;; int8 _mode
       (write-byte _mode s)
     ;; int8 _blendType
       (write-byte _blendType s)
     ;; int8 _sol
       (write-byte _sol s)
     ;; int8 _syncType
       (write-byte _syncType s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _vel
     (setq _vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _acc
     (setq _acc (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _radius
     (setq _radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;; int8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;; int8 _blendType
     (setq _blendType (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _blendType 127) (setq _blendType (- _blendType 256)))
   ;; int8 _sol
     (setq _sol (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _sol 127) (setq _sol (- _sol 256)))
   ;; int8 _syncType
     (setq _syncType (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _syncType 127) (setq _syncType (- _syncType 256)))
   ;;
   self)
  )

(defclass dsr_msgs::MoveJointxResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::MoveJointxResponse
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

(defclass dsr_msgs::MoveJointx
  :super ros::object
  :slots ())

(setf (get dsr_msgs::MoveJointx :md5sum-) "b9419696b37ce5162db29de34d49184f")
(setf (get dsr_msgs::MoveJointx :datatype-) "dsr_msgs/MoveJointx")
(setf (get dsr_msgs::MoveJointx :request) dsr_msgs::MoveJointxRequest)
(setf (get dsr_msgs::MoveJointx :response) dsr_msgs::MoveJointxResponse)

(defmethod dsr_msgs::MoveJointxRequest
  (:response () (instance dsr_msgs::MoveJointxResponse :init)))

(setf (get dsr_msgs::MoveJointxRequest :md5sum-) "b9419696b37ce5162db29de34d49184f")
(setf (get dsr_msgs::MoveJointxRequest :datatype-) "dsr_msgs/MoveJointxRequest")
(setf (get dsr_msgs::MoveJointxRequest :definition-)
      "#____________________________________________________________________________________________
# move_jointx  
#____________________________________________________________________________________________

float64[6] pos              # target  
float64    vel              # set velocity: [deg/sec]
float64    acc              # set acceleration: [deg/sec2] 
float64    time #= 0.0      # Time [sec] 
float64    radius #=0.0     # Radius under blending mode [mm]   
int8       ref              # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                            # <DR_WORLD is only available in M2.40 or later> 
int8       mode #= 0        # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
int8       blendType #= 0   # BLENDING_SPEED_TYPE_DUPLICATE=0, BLENDING_SPEED_TYPE_OVERRIDE=1
int8       sol              # SolutionSpace : 0~7
int8       syncType #=0     # SYNC = 0, ASYNC = 1
---
bool success
")

(setf (get dsr_msgs::MoveJointxResponse :md5sum-) "b9419696b37ce5162db29de34d49184f")
(setf (get dsr_msgs::MoveJointxResponse :datatype-) "dsr_msgs/MoveJointxResponse")
(setf (get dsr_msgs::MoveJointxResponse :definition-)
      "#____________________________________________________________________________________________
# move_jointx  
#____________________________________________________________________________________________

float64[6] pos              # target  
float64    vel              # set velocity: [deg/sec]
float64    acc              # set acceleration: [deg/sec2] 
float64    time #= 0.0      # Time [sec] 
float64    radius #=0.0     # Radius under blending mode [mm]   
int8       ref              # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                            # <DR_WORLD is only available in M2.40 or later> 
int8       mode #= 0        # MOVE_MODE_ABSOLUTE=0, MOVE_MODE_RELATIVE=1 
int8       blendType #= 0   # BLENDING_SPEED_TYPE_DUPLICATE=0, BLENDING_SPEED_TYPE_OVERRIDE=1
int8       sol              # SolutionSpace : 0~7
int8       syncType #=0     # SYNC = 0, ASYNC = 1
---
bool success
")



(provide :dsr_msgs/MoveJointx "b9419696b37ce5162db29de34d49184f")


