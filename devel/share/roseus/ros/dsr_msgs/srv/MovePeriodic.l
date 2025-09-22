;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::MovePeriodic)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'MovePeriodic (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::MOVEPERIODIC")
  (make-package "DSR_MSGS::MOVEPERIODIC"))
(unless (find-package "DSR_MSGS::MOVEPERIODICREQUEST")
  (make-package "DSR_MSGS::MOVEPERIODICREQUEST"))
(unless (find-package "DSR_MSGS::MOVEPERIODICRESPONSE")
  (make-package "DSR_MSGS::MOVEPERIODICRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::MovePeriodicRequest
  :super ros::object
  :slots (_amp _periodic _acc _repeat _ref _syncType ))

(defmethod dsr_msgs::MovePeriodicRequest
  (:init
   (&key
    ((:amp __amp) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:periodic __periodic) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:acc __acc) 0.0)
    ((:repeat __repeat) 0)
    ((:ref __ref) 0)
    ((:syncType __syncType) 0)
    )
   (send-super :init)
   (setq _amp __amp)
   (setq _periodic __periodic)
   (setq _acc (float __acc))
   (setq _repeat (round __repeat))
   (setq _ref (round __ref))
   (setq _syncType (round __syncType))
   self)
  (:amp
   (&optional __amp)
   (if __amp (setq _amp __amp)) _amp)
  (:periodic
   (&optional __periodic)
   (if __periodic (setq _periodic __periodic)) _periodic)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:repeat
   (&optional __repeat)
   (if __repeat (setq _repeat __repeat)) _repeat)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:syncType
   (&optional __syncType)
   (if __syncType (setq _syncType __syncType)) _syncType)
  (:serialization-length
   ()
   (+
    ;; float64[6] _amp
    (* 8    6)
    ;; float64[6] _periodic
    (* 8    6)
    ;; float64 _acc
    8
    ;; int8 _repeat
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
     ;; float64[6] _amp
     (dotimes (i 6)
       (sys::poke (elt _amp i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _periodic
     (dotimes (i 6)
       (sys::poke (elt _periodic i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _acc
       (sys::poke _acc (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8 _repeat
       (write-byte _repeat s)
     ;; int8 _ref
       (write-byte _ref s)
     ;; int8 _syncType
       (write-byte _syncType s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _amp
   (dotimes (i (length _amp))
     (setf (elt _amp i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _periodic
   (dotimes (i (length _periodic))
     (setf (elt _periodic i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _acc
     (setq _acc (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8 _repeat
     (setq _repeat (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _repeat 127) (setq _repeat (- _repeat 256)))
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;; int8 _syncType
     (setq _syncType (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _syncType 127) (setq _syncType (- _syncType 256)))
   ;;
   self)
  )

(defclass dsr_msgs::MovePeriodicResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::MovePeriodicResponse
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

(defclass dsr_msgs::MovePeriodic
  :super ros::object
  :slots ())

(setf (get dsr_msgs::MovePeriodic :md5sum-) "d18cadb0c21c585bdf0c4a1e6878f1a6")
(setf (get dsr_msgs::MovePeriodic :datatype-) "dsr_msgs/MovePeriodic")
(setf (get dsr_msgs::MovePeriodic :request) dsr_msgs::MovePeriodicRequest)
(setf (get dsr_msgs::MovePeriodic :response) dsr_msgs::MovePeriodicResponse)

(defmethod dsr_msgs::MovePeriodicRequest
  (:response () (instance dsr_msgs::MovePeriodicResponse :init)))

(setf (get dsr_msgs::MovePeriodicRequest :md5sum-) "d18cadb0c21c585bdf0c4a1e6878f1a6")
(setf (get dsr_msgs::MovePeriodicRequest :datatype-) "dsr_msgs/MovePeriodicRequest")
(setf (get dsr_msgs::MovePeriodicRequest :definition-)
      "#____________________________________________________________________________________________
# move_periodic  
#____________________________________________________________________________________________

float64[6] amp              # Amplitude (motion between -amp and +amp) [mm] or [deg]   
float64[6] periodic         # Period (time for 1 cycle) [sec]
float64    acc              # Acc-, dec- time [sec] 
int8       repeat           # Repetition count 
int8       ref  #= 1        # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                            # <DR_WORLD is only available in M2.40 or later 

int8       syncType #=0     # SYNC = 0, ASYNC = 1
---
bool success
")

(setf (get dsr_msgs::MovePeriodicResponse :md5sum-) "d18cadb0c21c585bdf0c4a1e6878f1a6")
(setf (get dsr_msgs::MovePeriodicResponse :datatype-) "dsr_msgs/MovePeriodicResponse")
(setf (get dsr_msgs::MovePeriodicResponse :definition-)
      "#____________________________________________________________________________________________
# move_periodic  
#____________________________________________________________________________________________

float64[6] amp              # Amplitude (motion between -amp and +amp) [mm] or [deg]   
float64[6] periodic         # Period (time for 1 cycle) [sec]
float64    acc              # Acc-, dec- time [sec] 
int8       repeat           # Repetition count 
int8       ref  #= 1        # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                            # <DR_WORLD is only available in M2.40 or later 

int8       syncType #=0     # SYNC = 0, ASYNC = 1
---
bool success
")



(provide :dsr_msgs/MovePeriodic "d18cadb0c21c585bdf0c4a1e6878f1a6")


