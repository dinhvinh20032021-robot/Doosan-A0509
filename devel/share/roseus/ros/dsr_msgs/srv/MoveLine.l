;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::MoveLine)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'MoveLine (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::MOVELINE")
  (make-package "DSR_MSGS::MOVELINE"))
(unless (find-package "DSR_MSGS::MOVELINEREQUEST")
  (make-package "DSR_MSGS::MOVELINEREQUEST"))
(unless (find-package "DSR_MSGS::MOVELINERESPONSE")
  (make-package "DSR_MSGS::MOVELINERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::MoveLineRequest
  :super ros::object
  :slots (_pos _vel _acc _time _radius _ref _mode _blendType _syncType ))

(defmethod dsr_msgs::MoveLineRequest
  (:init
   (&key
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:vel __vel) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:acc __acc) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:time __time) 0.0)
    ((:radius __radius) 0.0)
    ((:ref __ref) 0)
    ((:mode __mode) 0)
    ((:blendType __blendType) 0)
    ((:syncType __syncType) 0)
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _vel __vel)
   (setq _acc __acc)
   (setq _time (float __time))
   (setq _radius (float __radius))
   (setq _ref (round __ref))
   (setq _mode (round __mode))
   (setq _blendType (round __blendType))
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
  (:syncType
   (&optional __syncType)
   (if __syncType (setq _syncType __syncType)) _syncType)
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
    ;; float64 _radius
    8
    ;; int8 _ref
    1
    ;; int8 _mode
    1
    ;; int8 _blendType
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
     ;; float64 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8 _ref
       (write-byte _ref s)
     ;; int8 _mode
       (write-byte _mode s)
     ;; int8 _blendType
       (write-byte _blendType s)
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
   ;; int8 _syncType
     (setq _syncType (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _syncType 127) (setq _syncType (- _syncType 256)))
   ;;
   self)
  )

(defclass dsr_msgs::MoveLineResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::MoveLineResponse
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

(defclass dsr_msgs::MoveLine
  :super ros::object
  :slots ())

(setf (get dsr_msgs::MoveLine :md5sum-) "444a9dab874993faadb186c2ade7c1bc")
(setf (get dsr_msgs::MoveLine :datatype-) "dsr_msgs/MoveLine")
(setf (get dsr_msgs::MoveLine :request) dsr_msgs::MoveLineRequest)
(setf (get dsr_msgs::MoveLine :response) dsr_msgs::MoveLineResponse)

(defmethod dsr_msgs::MoveLineRequest
  (:response () (instance dsr_msgs::MoveLineResponse :init)))

(setf (get dsr_msgs::MoveLineRequest :md5sum-) "444a9dab874993faadb186c2ade7c1bc")
(setf (get dsr_msgs::MoveLineRequest :datatype-) "dsr_msgs/MoveLineRequest")
(setf (get dsr_msgs::MoveLineRequest :definition-)
      "#____________________________________________________________________________________________
# move_line  
#____________________________________________________________________________________________

float64[6] pos               # target  
float64[2] vel               # set velocity: [mm/sec], [deg/sec]
float64[2] acc               # set acceleration: [mm/sec2], [deg/sec2]
float64    time #= 0.0       # Time [sec] 
float64    radius #=0.0      # Radius under blending mode [mm] 
int8       ref               # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                             # <DR_WORLD is only available in M2.40 or later> 
int8       mode #= 0         # DR_MV_MOD_ABS(0), DR_MV_MOD_REL(1) 
int8       blendType #= 0    # BLENDING_SPEED_TYPE_DUPLICATE=0, BLENDING_SPEED_TYPE_OVERRIDE=1
int8       syncType #=0      # SYNC = 0, ASYNC = 1
---
bool success
")

(setf (get dsr_msgs::MoveLineResponse :md5sum-) "444a9dab874993faadb186c2ade7c1bc")
(setf (get dsr_msgs::MoveLineResponse :datatype-) "dsr_msgs/MoveLineResponse")
(setf (get dsr_msgs::MoveLineResponse :definition-)
      "#____________________________________________________________________________________________
# move_line  
#____________________________________________________________________________________________

float64[6] pos               # target  
float64[2] vel               # set velocity: [mm/sec], [deg/sec]
float64[2] acc               # set acceleration: [mm/sec2], [deg/sec2]
float64    time #= 0.0       # Time [sec] 
float64    radius #=0.0      # Radius under blending mode [mm] 
int8       ref               # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                             # <DR_WORLD is only available in M2.40 or later> 
int8       mode #= 0         # DR_MV_MOD_ABS(0), DR_MV_MOD_REL(1) 
int8       blendType #= 0    # BLENDING_SPEED_TYPE_DUPLICATE=0, BLENDING_SPEED_TYPE_OVERRIDE=1
int8       syncType #=0      # SYNC = 0, ASYNC = 1
---
bool success
")



(provide :dsr_msgs/MoveLine "444a9dab874993faadb186c2ade7c1bc")


