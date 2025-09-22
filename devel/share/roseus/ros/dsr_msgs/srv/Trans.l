;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::Trans)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'Trans (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::TRANS")
  (make-package "DSR_MSGS::TRANS"))
(unless (find-package "DSR_MSGS::TRANSREQUEST")
  (make-package "DSR_MSGS::TRANSREQUEST"))
(unless (find-package "DSR_MSGS::TRANSRESPONSE")
  (make-package "DSR_MSGS::TRANSRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::TransRequest
  :super ros::object
  :slots (_pos _delta _ref _ref_out ))

(defmethod dsr_msgs::TransRequest
  (:init
   (&key
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:delta __delta) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:ref __ref) 0)
    ((:ref_out __ref_out) 0)
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _delta __delta)
   (setq _ref (round __ref))
   (setq _ref_out (round __ref_out))
   self)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:delta
   (&optional __delta)
   (if __delta (setq _delta __delta)) _delta)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:ref_out
   (&optional __ref_out)
   (if __ref_out (setq _ref_out __ref_out)) _ref_out)
  (:serialization-length
   ()
   (+
    ;; float64[6] _pos
    (* 8    6)
    ;; float64[6] _delta
    (* 8    6)
    ;; int8 _ref
    1
    ;; int8 _ref_out
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
     ;; float64[6] _delta
     (dotimes (i 6)
       (sys::poke (elt _delta i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8 _ref
       (write-byte _ref s)
     ;; int8 _ref_out
       (write-byte _ref_out s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _delta
   (dotimes (i (length _delta))
     (setf (elt _delta i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;; int8 _ref_out
     (setq _ref_out (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref_out 127) (setq _ref_out (- _ref_out 256)))
   ;;
   self)
  )

(defclass dsr_msgs::TransResponse
  :super ros::object
  :slots (_trans_pos _success ))

(defmethod dsr_msgs::TransResponse
  (:init
   (&key
    ((:trans_pos __trans_pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _trans_pos __trans_pos)
   (setq _success __success)
   self)
  (:trans_pos
   (&optional __trans_pos)
   (if __trans_pos (setq _trans_pos __trans_pos)) _trans_pos)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float64[6] _trans_pos
    (* 8    6)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _trans_pos
     (dotimes (i 6)
       (sys::poke (elt _trans_pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _trans_pos
   (dotimes (i (length _trans_pos))
     (setf (elt _trans_pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::Trans
  :super ros::object
  :slots ())

(setf (get dsr_msgs::Trans :md5sum-) "2cc8472f65a9eb8b986e1489e30025bf")
(setf (get dsr_msgs::Trans :datatype-) "dsr_msgs/Trans")
(setf (get dsr_msgs::Trans :request) dsr_msgs::TransRequest)
(setf (get dsr_msgs::Trans :response) dsr_msgs::TransResponse)

(defmethod dsr_msgs::TransRequest
  (:response () (instance dsr_msgs::TransResponse :init)))

(setf (get dsr_msgs::TransRequest :md5sum-) "2cc8472f65a9eb8b986e1489e30025bf")
(setf (get dsr_msgs::TransRequest :datatype-) "dsr_msgs/TransRequest")
(setf (get dsr_msgs::TransRequest :definition-)
      "#____________________________________________________________________________________________
# trans  
#____________________________________________________________________________________________

float64[6] pos               # task pos(posx)  
float64[6] delta             # delta (posx)  
int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                             # <DR_WORLD is only available in M2.40 or later> 
int8       ref_out #= 0      # DR_BASE(0), DR_WORLD(2)
                             # <ref_out is only available in M2.40 or later>
---
float64[6] trans_pos         # trans pos(posx) 
bool       success
")

(setf (get dsr_msgs::TransResponse :md5sum-) "2cc8472f65a9eb8b986e1489e30025bf")
(setf (get dsr_msgs::TransResponse :datatype-) "dsr_msgs/TransResponse")
(setf (get dsr_msgs::TransResponse :definition-)
      "#____________________________________________________________________________________________
# trans  
#____________________________________________________________________________________________

float64[6] pos               # task pos(posx)  
float64[6] delta             # delta (posx)  
int8       ref     #= 0      # DR_BASE(0), DR_TOOL(1), DR_WORLD(2)
                             # <DR_WORLD is only available in M2.40 or later> 
int8       ref_out #= 0      # DR_BASE(0), DR_WORLD(2)
                             # <ref_out is only available in M2.40 or later>
---
float64[6] trans_pos         # trans pos(posx) 
bool       success
")



(provide :dsr_msgs/Trans "2cc8472f65a9eb8b986e1489e30025bf")


