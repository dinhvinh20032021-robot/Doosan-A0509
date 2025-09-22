;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::Fkin)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'Fkin (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::FKIN")
  (make-package "DSR_MSGS::FKIN"))
(unless (find-package "DSR_MSGS::FKINREQUEST")
  (make-package "DSR_MSGS::FKINREQUEST"))
(unless (find-package "DSR_MSGS::FKINRESPONSE")
  (make-package "DSR_MSGS::FKINRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::FkinRequest
  :super ros::object
  :slots (_pos _ref ))

(defmethod dsr_msgs::FkinRequest
  (:init
   (&key
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:ref __ref) 0)
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _ref (round __ref))
   self)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:serialization-length
   ()
   (+
    ;; float64[6] _pos
    (* 8    6)
    ;; int8 _ref
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
     ;; int8 _ref
       (write-byte _ref s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;;
   self)
  )

(defclass dsr_msgs::FkinResponse
  :super ros::object
  :slots (_conv_posx _success ))

(defmethod dsr_msgs::FkinResponse
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

(defclass dsr_msgs::Fkin
  :super ros::object
  :slots ())

(setf (get dsr_msgs::Fkin :md5sum-) "26c14bd0ea35ff7d87f4f121a2ae40ac")
(setf (get dsr_msgs::Fkin :datatype-) "dsr_msgs/Fkin")
(setf (get dsr_msgs::Fkin :request) dsr_msgs::FkinRequest)
(setf (get dsr_msgs::Fkin :response) dsr_msgs::FkinResponse)

(defmethod dsr_msgs::FkinRequest
  (:response () (instance dsr_msgs::FkinResponse :init)))

(setf (get dsr_msgs::FkinRequest :md5sum-) "26c14bd0ea35ff7d87f4f121a2ae40ac")
(setf (get dsr_msgs::FkinRequest :datatype-) "dsr_msgs/FkinRequest")
(setf (get dsr_msgs::FkinRequest :definition-)
      "#____________________________________________________________________________________________
# fkin  
#____________________________________________________________________________________________

float64[6] pos               # joint pos(posj)  
int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)
                             # <ref is only available in M2.40 or later> 
---
float64[6] conv_posx         # task pos(posx)
bool       success 
")

(setf (get dsr_msgs::FkinResponse :md5sum-) "26c14bd0ea35ff7d87f4f121a2ae40ac")
(setf (get dsr_msgs::FkinResponse :datatype-) "dsr_msgs/FkinResponse")
(setf (get dsr_msgs::FkinResponse :definition-)
      "#____________________________________________________________________________________________
# fkin  
#____________________________________________________________________________________________

float64[6] pos               # joint pos(posj)  
int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)
                             # <ref is only available in M2.40 or later> 
---
float64[6] conv_posx         # task pos(posx)
bool       success 
")



(provide :dsr_msgs/Fkin "26c14bd0ea35ff7d87f4f121a2ae40ac")


