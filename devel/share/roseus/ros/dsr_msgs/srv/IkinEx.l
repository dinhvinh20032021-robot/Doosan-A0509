;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::IkinEx)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'IkinEx (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::IKINEX")
  (make-package "DSR_MSGS::IKINEX"))
(unless (find-package "DSR_MSGS::IKINEXREQUEST")
  (make-package "DSR_MSGS::IKINEXREQUEST"))
(unless (find-package "DSR_MSGS::IKINEXRESPONSE")
  (make-package "DSR_MSGS::IKINEXRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::IkinExRequest
  :super ros::object
  :slots (_pos _sol_space _ref _ref_pos_opt ))

(defmethod dsr_msgs::IkinExRequest
  (:init
   (&key
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:sol_space __sol_space) 0)
    ((:ref __ref) 0)
    ((:ref_pos_opt __ref_pos_opt) 0)
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _sol_space (round __sol_space))
   (setq _ref (round __ref))
   (setq _ref_pos_opt (round __ref_pos_opt))
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
  (:ref_pos_opt
   (&optional __ref_pos_opt)
   (if __ref_pos_opt (setq _ref_pos_opt __ref_pos_opt)) _ref_pos_opt)
  (:serialization-length
   ()
   (+
    ;; float64[6] _pos
    (* 8    6)
    ;; int8 _sol_space
    1
    ;; int8 _ref
    1
    ;; int8 _ref_pos_opt
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
     ;; int8 _ref_pos_opt
       (write-byte _ref_pos_opt s)
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
   ;; int8 _ref_pos_opt
     (setq _ref_pos_opt (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref_pos_opt 127) (setq _ref_pos_opt (- _ref_pos_opt 256)))
   ;;
   self)
  )

(defclass dsr_msgs::IkinExResponse
  :super ros::object
  :slots (_conv_posj _status _success ))

(defmethod dsr_msgs::IkinExResponse
  (:init
   (&key
    ((:conv_posj __conv_posj) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:status __status) nil)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _conv_posj __conv_posj)
   (setq _status __status)
   (setq _success __success)
   self)
  (:conv_posj
   (&optional __conv_posj)
   (if __conv_posj (setq _conv_posj __conv_posj)) _conv_posj)
  (:status
   (&optional (__status :null))
   (if (not (eq __status :null)) (setq _status __status)) _status)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float64[6] _conv_posj
    (* 8    6)
    ;; bool _status
    1
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
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
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
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::IkinEx
  :super ros::object
  :slots ())

(setf (get dsr_msgs::IkinEx :md5sum-) "52c2edae66404c90e27c09703f4690c1")
(setf (get dsr_msgs::IkinEx :datatype-) "dsr_msgs/IkinEx")
(setf (get dsr_msgs::IkinEx :request) dsr_msgs::IkinExRequest)
(setf (get dsr_msgs::IkinEx :response) dsr_msgs::IkinExResponse)

(defmethod dsr_msgs::IkinExRequest
  (:response () (instance dsr_msgs::IkinExResponse :init)))

(setf (get dsr_msgs::IkinExRequest :md5sum-) "52c2edae66404c90e27c09703f4690c1")
(setf (get dsr_msgs::IkinExRequest :datatype-) "dsr_msgs/IkinExRequest")
(setf (get dsr_msgs::IkinExRequest :definition-)
      "#____________________________________________________________________________________________
# ikin_ex  
#____________________________________________________________________________________________

float64[6] pos               # task pos(posx)  
int8       sol_space         # solution space : 0 ~ 7
int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)
int8       ref_pos_opt	     #  
---
float64[6] conv_posj         # joint pos(posj)  
bool       status
bool       success

")

(setf (get dsr_msgs::IkinExResponse :md5sum-) "52c2edae66404c90e27c09703f4690c1")
(setf (get dsr_msgs::IkinExResponse :datatype-) "dsr_msgs/IkinExResponse")
(setf (get dsr_msgs::IkinExResponse :definition-)
      "#____________________________________________________________________________________________
# ikin_ex  
#____________________________________________________________________________________________

float64[6] pos               # task pos(posx)  
int8       sol_space         # solution space : 0 ~ 7
int8       ref     #= 0      # DR_BASE(0), DR_WORLD(2)
int8       ref_pos_opt	     #  
---
float64[6] conv_posj         # joint pos(posj)  
bool       status
bool       success

")



(provide :dsr_msgs/IkinEx "52c2edae66404c90e27c09703f4690c1")


