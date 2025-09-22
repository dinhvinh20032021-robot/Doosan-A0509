;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetCurrentPosj)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetCurrentPosj (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETCURRENTPOSJ")
  (make-package "DSR_MSGS::GETCURRENTPOSJ"))
(unless (find-package "DSR_MSGS::GETCURRENTPOSJREQUEST")
  (make-package "DSR_MSGS::GETCURRENTPOSJREQUEST"))
(unless (find-package "DSR_MSGS::GETCURRENTPOSJRESPONSE")
  (make-package "DSR_MSGS::GETCURRENTPOSJRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetCurrentPosjRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::GetCurrentPosjRequest
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

(defclass dsr_msgs::GetCurrentPosjResponse
  :super ros::object
  :slots (_pos _success ))

(defmethod dsr_msgs::GetCurrentPosjResponse
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

(defclass dsr_msgs::GetCurrentPosj
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetCurrentPosj :md5sum-) "3d228bb409a5fe0306e199313e3ed277")
(setf (get dsr_msgs::GetCurrentPosj :datatype-) "dsr_msgs/GetCurrentPosj")
(setf (get dsr_msgs::GetCurrentPosj :request) dsr_msgs::GetCurrentPosjRequest)
(setf (get dsr_msgs::GetCurrentPosj :response) dsr_msgs::GetCurrentPosjResponse)

(defmethod dsr_msgs::GetCurrentPosjRequest
  (:response () (instance dsr_msgs::GetCurrentPosjResponse :init)))

(setf (get dsr_msgs::GetCurrentPosjRequest :md5sum-) "3d228bb409a5fe0306e199313e3ed277")
(setf (get dsr_msgs::GetCurrentPosjRequest :datatype-) "dsr_msgs/GetCurrentPosjRequest")
(setf (get dsr_msgs::GetCurrentPosjRequest :definition-)
      "#____________________________________________________________________________________________
# get_current_posj()  
#____________________________________________________________________________________________
# This service returns the current joint angle.

---
float64[6] pos               # joint pos(posj)  
bool       success
")

(setf (get dsr_msgs::GetCurrentPosjResponse :md5sum-) "3d228bb409a5fe0306e199313e3ed277")
(setf (get dsr_msgs::GetCurrentPosjResponse :datatype-) "dsr_msgs/GetCurrentPosjResponse")
(setf (get dsr_msgs::GetCurrentPosjResponse :definition-)
      "#____________________________________________________________________________________________
# get_current_posj()  
#____________________________________________________________________________________________
# This service returns the current joint angle.

---
float64[6] pos               # joint pos(posj)  
bool       success
")



(provide :dsr_msgs/GetCurrentPosj "3d228bb409a5fe0306e199313e3ed277")


