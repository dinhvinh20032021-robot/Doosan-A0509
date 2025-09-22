;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetDesiredPosj)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetDesiredPosj (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETDESIREDPOSJ")
  (make-package "DSR_MSGS::GETDESIREDPOSJ"))
(unless (find-package "DSR_MSGS::GETDESIREDPOSJREQUEST")
  (make-package "DSR_MSGS::GETDESIREDPOSJREQUEST"))
(unless (find-package "DSR_MSGS::GETDESIREDPOSJRESPONSE")
  (make-package "DSR_MSGS::GETDESIREDPOSJRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetDesiredPosjRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetDesiredPosjRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass dsr_msgs::GetDesiredPosjResponse
  :super ros::object
  :slots (_pos _success ))

(defmethod dsr_msgs::GetDesiredPosjResponse
  (:init
   (&key
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _success __success)
   self)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float64[6] _pos
    (* 8    6)
    ;; bool _success
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
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetDesiredPosj
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetDesiredPosj :md5sum-) "3d228bb409a5fe0306e199313e3ed277")
(setf (get dsr_msgs::GetDesiredPosj :datatype-) "dsr_msgs/GetDesiredPosj")
(setf (get dsr_msgs::GetDesiredPosj :request) dsr_msgs::GetDesiredPosjRequest)
(setf (get dsr_msgs::GetDesiredPosj :response) dsr_msgs::GetDesiredPosjResponse)

(defmethod dsr_msgs::GetDesiredPosjRequest
  (:response () (instance dsr_msgs::GetDesiredPosjResponse :init)))

(setf (get dsr_msgs::GetDesiredPosjRequest :md5sum-) "3d228bb409a5fe0306e199313e3ed277")
(setf (get dsr_msgs::GetDesiredPosjRequest :datatype-) "dsr_msgs/GetDesiredPosjRequest")
(setf (get dsr_msgs::GetDesiredPosjRequest :definition-)
      "#____________________________________________________________________________________________
# get_desired_posj()  
#____________________________________________________________________________________________
# This service returns the current target joint angle.
# It cannot be used in the movel, movec, movesx, moveb, move_spiral, or move_periodic service.

---
float64[6] pos               # joint pos(posj)  
bool       success
")

(setf (get dsr_msgs::GetDesiredPosjResponse :md5sum-) "3d228bb409a5fe0306e199313e3ed277")
(setf (get dsr_msgs::GetDesiredPosjResponse :datatype-) "dsr_msgs/GetDesiredPosjResponse")
(setf (get dsr_msgs::GetDesiredPosjResponse :definition-)
      "#____________________________________________________________________________________________
# get_desired_posj()  
#____________________________________________________________________________________________
# This service returns the current target joint angle.
# It cannot be used in the movel, movec, movesx, moveb, move_spiral, or move_periodic service.

---
float64[6] pos               # joint pos(posj)  
bool       success
")



(provide :dsr_msgs/GetDesiredPosj "3d228bb409a5fe0306e199313e3ed277")


