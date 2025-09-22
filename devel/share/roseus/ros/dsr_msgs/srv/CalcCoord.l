;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::CalcCoord)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'CalcCoord (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::CALCCOORD")
  (make-package "DSR_MSGS::CALCCOORD"))
(unless (find-package "DSR_MSGS::CALCCOORDREQUEST")
  (make-package "DSR_MSGS::CALCCOORDREQUEST"))
(unless (find-package "DSR_MSGS::CALCCOORDRESPONSE")
  (make-package "DSR_MSGS::CALCCOORDRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::CalcCoordRequest
  :super ros::object
  :slots (_input_pos_cnt _x1 _x2 _x3 _x4 _ref _mod ))

(defmethod dsr_msgs::CalcCoordRequest
  (:init
   (&key
    ((:input_pos_cnt __input_pos_cnt) 0)
    ((:x1 __x1) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:x2 __x2) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:x3 __x3) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:x4 __x4) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:ref __ref) 0)
    ((:mod __mod) 0)
    )
   (send-super :init)
   (setq _input_pos_cnt (round __input_pos_cnt))
   (setq _x1 __x1)
   (setq _x2 __x2)
   (setq _x3 __x3)
   (setq _x4 __x4)
   (setq _ref (round __ref))
   (setq _mod (round __mod))
   self)
  (:input_pos_cnt
   (&optional __input_pos_cnt)
   (if __input_pos_cnt (setq _input_pos_cnt __input_pos_cnt)) _input_pos_cnt)
  (:x1
   (&optional __x1)
   (if __x1 (setq _x1 __x1)) _x1)
  (:x2
   (&optional __x2)
   (if __x2 (setq _x2 __x2)) _x2)
  (:x3
   (&optional __x3)
   (if __x3 (setq _x3 __x3)) _x3)
  (:x4
   (&optional __x4)
   (if __x4 (setq _x4 __x4)) _x4)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:mod
   (&optional __mod)
   (if __mod (setq _mod __mod)) _mod)
  (:serialization-length
   ()
   (+
    ;; int8 _input_pos_cnt
    1
    ;; float64[6] _x1
    (* 8    6)
    ;; float64[6] _x2
    (* 8    6)
    ;; float64[6] _x3
    (* 8    6)
    ;; float64[6] _x4
    (* 8    6)
    ;; int8 _ref
    1
    ;; int8 _mod
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _input_pos_cnt
       (write-byte _input_pos_cnt s)
     ;; float64[6] _x1
     (dotimes (i 6)
       (sys::poke (elt _x1 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _x2
     (dotimes (i 6)
       (sys::poke (elt _x2 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _x3
     (dotimes (i 6)
       (sys::poke (elt _x3 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _x4
     (dotimes (i 6)
       (sys::poke (elt _x4 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8 _ref
       (write-byte _ref s)
     ;; int8 _mod
       (write-byte _mod s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _input_pos_cnt
     (setq _input_pos_cnt (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _input_pos_cnt 127) (setq _input_pos_cnt (- _input_pos_cnt 256)))
   ;; float64[6] _x1
   (dotimes (i (length _x1))
     (setf (elt _x1 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _x2
   (dotimes (i (length _x2))
     (setf (elt _x2 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _x3
   (dotimes (i (length _x3))
     (setf (elt _x3 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _x4
   (dotimes (i (length _x4))
     (setf (elt _x4 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;; int8 _mod
     (setq _mod (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mod 127) (setq _mod (- _mod 256)))
   ;;
   self)
  )

(defclass dsr_msgs::CalcCoordResponse
  :super ros::object
  :slots (_conv_posx _success ))

(defmethod dsr_msgs::CalcCoordResponse
  (:init
   (&key
    ((:conv_posx __conv_posx) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _conv_posx __conv_posx)
   (setq _success __success)
   self)
  (:conv_posx
   (&optional __conv_posx)
   (if __conv_posx (setq _conv_posx __conv_posx)) _conv_posx)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float64[6] _conv_posx
    (* 8    6)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _conv_posx
     (dotimes (i 6)
       (sys::poke (elt _conv_posx i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _conv_posx
   (dotimes (i (length _conv_posx))
     (setf (elt _conv_posx i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::CalcCoord
  :super ros::object
  :slots ())

(setf (get dsr_msgs::CalcCoord :md5sum-) "2001e63c6ceb43ba2ea8431cc72302b9")
(setf (get dsr_msgs::CalcCoord :datatype-) "dsr_msgs/CalcCoord")
(setf (get dsr_msgs::CalcCoord :request) dsr_msgs::CalcCoordRequest)
(setf (get dsr_msgs::CalcCoord :response) dsr_msgs::CalcCoordResponse)

(defmethod dsr_msgs::CalcCoordRequest
  (:response () (instance dsr_msgs::CalcCoordResponse :init)))

(setf (get dsr_msgs::CalcCoordRequest :md5sum-) "2001e63c6ceb43ba2ea8431cc72302b9")
(setf (get dsr_msgs::CalcCoordRequest :datatype-) "dsr_msgs/CalcCoordRequest")
(setf (get dsr_msgs::CalcCoordRequest :definition-)
      "#____________________________________________________________________________________________
# calc_coord   
#____________________________________________________________________________________________
# This service is only available in M2.50 or later

int8       input_pos_cnt     # input_pos_cnt
float64[6] x1                # task pos(posx)  
float64[6] x2                # task pos(posx)  
float64[6] x3                # task pos(posx)
float64[6] x4                # task pos(posx)
int8       ref               # DR_BASE(0), DR_WORLD(2)
int8       mod               # input mode(only valid when the number of input poses is 2)
                             # 0: defining z-axis based on the current Tool-z direction
                             # 1: defining z-axis based on the z direction of x1 
---
float64[6] conv_posx         # task pos(posx) 
bool       success
")

(setf (get dsr_msgs::CalcCoordResponse :md5sum-) "2001e63c6ceb43ba2ea8431cc72302b9")
(setf (get dsr_msgs::CalcCoordResponse :datatype-) "dsr_msgs/CalcCoordResponse")
(setf (get dsr_msgs::CalcCoordResponse :definition-)
      "#____________________________________________________________________________________________
# calc_coord   
#____________________________________________________________________________________________
# This service is only available in M2.50 or later

int8       input_pos_cnt     # input_pos_cnt
float64[6] x1                # task pos(posx)  
float64[6] x2                # task pos(posx)  
float64[6] x3                # task pos(posx)
float64[6] x4                # task pos(posx)
int8       ref               # DR_BASE(0), DR_WORLD(2)
int8       mod               # input mode(only valid when the number of input poses is 2)
                             # 0: defining z-axis based on the current Tool-z direction
                             # 1: defining z-axis based on the z direction of x1 
---
float64[6] conv_posx         # task pos(posx) 
bool       success
")



(provide :dsr_msgs/CalcCoord "2001e63c6ceb43ba2ea8431cc72302b9")


