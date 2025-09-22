;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetRefCoord)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetRefCoord (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETREFCOORD")
  (make-package "DSR_MSGS::SETREFCOORD"))
(unless (find-package "DSR_MSGS::SETREFCOORDREQUEST")
  (make-package "DSR_MSGS::SETREFCOORDREQUEST"))
(unless (find-package "DSR_MSGS::SETREFCOORDRESPONSE")
  (make-package "DSR_MSGS::SETREFCOORDRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetRefCoordRequest
  :super ros::object
  :slots (_coord ))

(defmethod dsr_msgs::SetRefCoordRequest
  (:init
   (&key
    ((:coord __coord) 0)
    )
   (send-super :init)
   (setq _coord (round __coord))
   self)
  (:coord
   (&optional __coord)
   (if __coord (setq _coord __coord)) _coord)
  (:serialization-length
   ()
   (+
    ;; int8 _coord
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _coord
       (write-byte _coord s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _coord
     (setq _coord (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _coord 127) (setq _coord (- _coord 256)))
   ;;
   self)
  )

(defclass dsr_msgs::SetRefCoordResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetRefCoordResponse
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

(defclass dsr_msgs::SetRefCoord
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetRefCoord :md5sum-) "1e3275b6f46c8260502524b4e6bedf9c")
(setf (get dsr_msgs::SetRefCoord :datatype-) "dsr_msgs/SetRefCoord")
(setf (get dsr_msgs::SetRefCoord :request) dsr_msgs::SetRefCoordRequest)
(setf (get dsr_msgs::SetRefCoord :response) dsr_msgs::SetRefCoordResponse)

(defmethod dsr_msgs::SetRefCoordRequest
  (:response () (instance dsr_msgs::SetRefCoordResponse :init)))

(setf (get dsr_msgs::SetRefCoordRequest :md5sum-) "1e3275b6f46c8260502524b4e6bedf9c")
(setf (get dsr_msgs::SetRefCoordRequest :datatype-) "dsr_msgs/SetRefCoordRequest")
(setf (get dsr_msgs::SetRefCoordRequest :definition-)
      "#____________________________________________________________________________________________
# set_ref_coord 
#____________________________________________________________________________________________

int8       coord            # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200)
                            # <DR_WORLD is only available in M2.40 or later> 
---
bool success
")

(setf (get dsr_msgs::SetRefCoordResponse :md5sum-) "1e3275b6f46c8260502524b4e6bedf9c")
(setf (get dsr_msgs::SetRefCoordResponse :datatype-) "dsr_msgs/SetRefCoordResponse")
(setf (get dsr_msgs::SetRefCoordResponse :definition-)
      "#____________________________________________________________________________________________
# set_ref_coord 
#____________________________________________________________________________________________

int8       coord            # DR_BASE(0), DR_TOOL(1), DR_WORLD(2), user coord(101~200)
                            # <DR_WORLD is only available in M2.40 or later> 
---
bool success
")



(provide :dsr_msgs/SetRefCoord "1e3275b6f46c8260502524b4e6bedf9c")


