;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::Ikin)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'Ikin (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::IKIN")
  (make-package "DSR_MSGS::IKIN"))
(unless (find-package "DSR_MSGS::IKINREQUEST")
  (make-package "DSR_MSGS::IKINREQUEST"))
(unless (find-package "DSR_MSGS::IKINRESPONSE")
  (make-package "DSR_MSGS::IKINRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::IkinRequest
  :super ros::object
  :slots (_pos _sol_space _ref ))

(defmethod dsr_msgs::IkinRequest
  (:init
   (&key
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:sol_space __sol_space) 0)
    ((:ref __ref) 0)
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _sol_space (round __sol_space))
   (setq _ref (round __ref))
   self)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:sol_space
   (&optional __sol_space)
   (if __sol_space (setq _sol_space __sol_space)) _sol_space)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:serialization-length
   ()
   (+
    ;; float64[6] _pos
    (* 8    6)
    ;; int8 _sol_space
    1
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
     ;; int8 _sol_space
       (write-byte _sol_space s)
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
   ;; int8 _sol_space
     (setq _sol_space (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _sol_space 127) (setq _sol_space (- _sol_space 256)))
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;;
   self)
  )

(defclass dsr_msgs::IkinResponse
  :super ros::object
  :slots (_conv_posj _success ))

(defmethod dsr_msgs::IkinResponse
  (:init
   (&key
    ((:conv_posj __conv_posj) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _conv_posj __conv_posj)
   (setq _success __success)
   self)
  (:conv_posj
   (&optional __conv_posj)
   (if __conv_posj (setq _conv_posj __conv_posj)) _conv_posj)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float64[6] _conv_posj
    (* 8    6)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _conv_posj
     (dotimes (i 6)
       (sys::poke (elt _conv_posj i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _conv_posj
   (dotimes (i (length _conv_posj))
     (setf (elt _conv_posj i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::Ikin
  :super ros::object
  :slots ())

(setf (get dsr_msgs::Ikin :md5sum-) "fb475e378a19f7cef6d8ac54d5b6fb72")
(setf (get dsr_msgs::Ikin :datatype-) "dsr_msgs/Ikin")
(setf (get dsr_msgs::Ikin :request) dsr_msgs::IkinRequest)
(setf (get dsr_msgs::Ikin :response) dsr_msgs::IkinResponse)

(defmethod dsr_msgs::IkinRequest
  (:response () (instance dsr_msgs::IkinResponse :init)))

(setf (get dsr_msgs::IkinRequest :md5sum-) "fb475e378a19f7cef6d8ac54d5b6fb72")
(setf (get dsr_msgs::IkinRequest :datatype-) "dsr_msgs/IkinRequest")
(setf (get dsr_msgs::IkinRequest :definition-)
      "#____________________________________________________________________________________________
# ikin  
#____________________________________________________________________________________________

float64[6] pos               # task pos(posx)  
int8       sol_space         # solution space : 0 ~ 7
int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)
                             # <ref is only available in M2.40 or later> 
---
float64[6] conv_posj         # joint pos(posj)  
bool       success
")

(setf (get dsr_msgs::IkinResponse :md5sum-) "fb475e378a19f7cef6d8ac54d5b6fb72")
(setf (get dsr_msgs::IkinResponse :datatype-) "dsr_msgs/IkinResponse")
(setf (get dsr_msgs::IkinResponse :definition-)
      "#____________________________________________________________________________________________
# ikin  
#____________________________________________________________________________________________

float64[6] pos               # task pos(posx)  
int8       sol_space         # solution space : 0 ~ 7
int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)
                             # <ref is only available in M2.40 or later> 
---
float64[6] conv_posj         # joint pos(posj)  
bool       success
")



(provide :dsr_msgs/Ikin "fb475e378a19f7cef6d8ac54d5b6fb72")


