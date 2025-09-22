;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetDesiredForce)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetDesiredForce (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETDESIREDFORCE")
  (make-package "DSR_MSGS::SETDESIREDFORCE"))
(unless (find-package "DSR_MSGS::SETDESIREDFORCEREQUEST")
  (make-package "DSR_MSGS::SETDESIREDFORCEREQUEST"))
(unless (find-package "DSR_MSGS::SETDESIREDFORCERESPONSE")
  (make-package "DSR_MSGS::SETDESIREDFORCERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetDesiredForceRequest
  :super ros::object
  :slots (_fd _dir _ref _time _mod ))

(defmethod dsr_msgs::SetDesiredForceRequest
  (:init
   (&key
    ((:fd __fd) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:dir __dir) (make-array 6 :initial-element 0 :element-type :integer))
    ((:ref __ref) 0)
    ((:time __time) 0.0)
    ((:mod __mod) 0)
    )
   (send-super :init)
   (setq _fd __fd)
   (setq _dir __dir)
   (setq _ref (round __ref))
   (setq _time (float __time))
   (setq _mod (round __mod))
   self)
  (:fd
   (&optional __fd)
   (if __fd (setq _fd __fd)) _fd)
  (:dir
   (&optional __dir)
   (if __dir (setq _dir __dir)) _dir)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:mod
   (&optional __mod)
   (if __mod (setq _mod __mod)) _mod)
  (:serialization-length
   ()
   (+
    ;; float64[6] _fd
    (* 8    6)
    ;; int8[6] _dir
    (* 1    6)
    ;; int8 _ref
    1
    ;; float64 _time
    8
    ;; int8 _mod
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _fd
     (dotimes (i 6)
       (sys::poke (elt _fd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8[6] _dir
     (dotimes (i 6)
       (write-byte (elt _dir i) s)
       )
     ;; int8 _ref
       (write-byte _ref s)
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8 _mod
       (write-byte _mod s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _fd
   (dotimes (i (length _fd))
     (setf (elt _fd i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int8[6] _dir
   (dotimes (i (length _dir))
     (setf (elt _dir i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _dir i) 127) (setf (elt _dir i) (- (elt _dir i) 256)))
     )
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8 _mod
     (setq _mod (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mod 127) (setq _mod (- _mod 256)))
   ;;
   self)
  )

(defclass dsr_msgs::SetDesiredForceResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetDesiredForceResponse
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

(defclass dsr_msgs::SetDesiredForce
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetDesiredForce :md5sum-) "06a47c6fc3ae37ec04174ee64d651e04")
(setf (get dsr_msgs::SetDesiredForce :datatype-) "dsr_msgs/SetDesiredForce")
(setf (get dsr_msgs::SetDesiredForce :request) dsr_msgs::SetDesiredForceRequest)
(setf (get dsr_msgs::SetDesiredForce :response) dsr_msgs::SetDesiredForceResponse)

(defmethod dsr_msgs::SetDesiredForceRequest
  (:response () (instance dsr_msgs::SetDesiredForceResponse :init)))

(setf (get dsr_msgs::SetDesiredForceRequest :md5sum-) "06a47c6fc3ae37ec04174ee64d651e04")
(setf (get dsr_msgs::SetDesiredForceRequest :datatype-) "dsr_msgs/SetDesiredForceRequest")
(setf (get dsr_msgs::SetDesiredForceRequest :definition-)
      "#____________________________________________________________________________________________
# set_desired_force  
#____________________________________________________________________________________________

float64[6] fd                # Three translational target forces + Three rotational target moments
int8[6]    dir               # Force control in the corresponding direction if 1, Compliance control in the corresponding direction if 0
int8       ref               # Reference coordinate of the coordinate to get
float64    time # 0          # Transition time of target force to take effect (0 ~ 1.0 sec)
int8       mod               # DR_FC_MOD_ABS(0): force control with absolute value, 
                             # DR_FC_MOD_REL(1): force control with relative value to initial state (the instance when this function is called) 
---
bool       success
")

(setf (get dsr_msgs::SetDesiredForceResponse :md5sum-) "06a47c6fc3ae37ec04174ee64d651e04")
(setf (get dsr_msgs::SetDesiredForceResponse :datatype-) "dsr_msgs/SetDesiredForceResponse")
(setf (get dsr_msgs::SetDesiredForceResponse :definition-)
      "#____________________________________________________________________________________________
# set_desired_force  
#____________________________________________________________________________________________

float64[6] fd                # Three translational target forces + Three rotational target moments
int8[6]    dir               # Force control in the corresponding direction if 1, Compliance control in the corresponding direction if 0
int8       ref               # Reference coordinate of the coordinate to get
float64    time # 0          # Transition time of target force to take effect (0 ~ 1.0 sec)
int8       mod               # DR_FC_MOD_ABS(0): force control with absolute value, 
                             # DR_FC_MOD_REL(1): force control with relative value to initial state (the instance when this function is called) 
---
bool       success
")



(provide :dsr_msgs/SetDesiredForce "06a47c6fc3ae37ec04174ee64d651e04")


