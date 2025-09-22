;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetSolutionSpace)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetSolutionSpace (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETSOLUTIONSPACE")
  (make-package "DSR_MSGS::GETSOLUTIONSPACE"))
(unless (find-package "DSR_MSGS::GETSOLUTIONSPACEREQUEST")
  (make-package "DSR_MSGS::GETSOLUTIONSPACEREQUEST"))
(unless (find-package "DSR_MSGS::GETSOLUTIONSPACERESPONSE")
  (make-package "DSR_MSGS::GETSOLUTIONSPACERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetSolutionSpaceRequest
  :super ros::object
  :slots (_pos ))

(defmethod dsr_msgs::GetSolutionSpaceRequest
  (:init
   (&key
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _pos __pos)
   self)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:serialization-length
   ()
   (+
    ;; float64[6] _pos
    (* 8    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _pos
     (dotimes (i 6)
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass dsr_msgs::GetSolutionSpaceResponse
  :super ros::object
  :slots (_sol_space _success ))

(defmethod dsr_msgs::GetSolutionSpaceResponse
  (:init
   (&key
    ((:sol_space __sol_space) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _sol_space (round __sol_space))
   (setq _success __success)
   self)
  (:sol_space
   (&optional __sol_space)
   (if __sol_space (setq _sol_space __sol_space)) _sol_space)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int8 _sol_space
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _sol_space
       (write-byte _sol_space s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _sol_space
     (setq _sol_space (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _sol_space 127) (setq _sol_space (- _sol_space 256)))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetSolutionSpace
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetSolutionSpace :md5sum-) "27abbe19a7d41c61a3226076e3320ebf")
(setf (get dsr_msgs::GetSolutionSpace :datatype-) "dsr_msgs/GetSolutionSpace")
(setf (get dsr_msgs::GetSolutionSpace :request) dsr_msgs::GetSolutionSpaceRequest)
(setf (get dsr_msgs::GetSolutionSpace :response) dsr_msgs::GetSolutionSpaceResponse)

(defmethod dsr_msgs::GetSolutionSpaceRequest
  (:response () (instance dsr_msgs::GetSolutionSpaceResponse :init)))

(setf (get dsr_msgs::GetSolutionSpaceRequest :md5sum-) "27abbe19a7d41c61a3226076e3320ebf")
(setf (get dsr_msgs::GetSolutionSpaceRequest :datatype-) "dsr_msgs/GetSolutionSpaceRequest")
(setf (get dsr_msgs::GetSolutionSpaceRequest :definition-)
      "#____________________________________________________________________________________________
# get_solution_space(pos)  
#____________________________________________________________________________________________
# This service obtains the solution space value.

float64[6] pos               # joint angle list [degree] 
---
int8       sol_space         # solution space : 0 ~ 7
bool       success

")

(setf (get dsr_msgs::GetSolutionSpaceResponse :md5sum-) "27abbe19a7d41c61a3226076e3320ebf")
(setf (get dsr_msgs::GetSolutionSpaceResponse :datatype-) "dsr_msgs/GetSolutionSpaceResponse")
(setf (get dsr_msgs::GetSolutionSpaceResponse :definition-)
      "#____________________________________________________________________________________________
# get_solution_space(pos)  
#____________________________________________________________________________________________
# This service obtains the solution space value.

float64[6] pos               # joint angle list [degree] 
---
int8       sol_space         # solution space : 0 ~ 7
bool       success

")



(provide :dsr_msgs/GetSolutionSpace "27abbe19a7d41c61a3226076e3320ebf")


