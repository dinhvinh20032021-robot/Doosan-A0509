;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetCurrentPose)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetCurrentPose (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETCURRENTPOSE")
  (make-package "DSR_MSGS::GETCURRENTPOSE"))
(unless (find-package "DSR_MSGS::GETCURRENTPOSEREQUEST")
  (make-package "DSR_MSGS::GETCURRENTPOSEREQUEST"))
(unless (find-package "DSR_MSGS::GETCURRENTPOSERESPONSE")
  (make-package "DSR_MSGS::GETCURRENTPOSERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetCurrentPoseRequest
  :super ros::object
  :slots (_space_type ))

(defmethod dsr_msgs::GetCurrentPoseRequest
  (:init
   (&key
    ((:space_type __space_type) 0)
    )
   (send-super :init)
   (setq _space_type (round __space_type))
   self)
  (:space_type
   (&optional __space_type)
   (if __space_type (setq _space_type __space_type)) _space_type)
  (:serialization-length
   ()
   (+
    ;; int8 _space_type
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _space_type
       (write-byte _space_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _space_type
     (setq _space_type (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _space_type 127) (setq _space_type (- _space_type 256)))
   ;;
   self)
  )

(defclass dsr_msgs::GetCurrentPoseResponse
  :super ros::object
  :slots (_pos _success ))

(defmethod dsr_msgs::GetCurrentPoseResponse
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

(defclass dsr_msgs::GetCurrentPose
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetCurrentPose :md5sum-) "7d90ac1bbfd6c3d94c5e9258853f72a9")
(setf (get dsr_msgs::GetCurrentPose :datatype-) "dsr_msgs/GetCurrentPose")
(setf (get dsr_msgs::GetCurrentPose :request) dsr_msgs::GetCurrentPoseRequest)
(setf (get dsr_msgs::GetCurrentPose :response) dsr_msgs::GetCurrentPoseResponse)

(defmethod dsr_msgs::GetCurrentPoseRequest
  (:response () (instance dsr_msgs::GetCurrentPoseResponse :init)))

(setf (get dsr_msgs::GetCurrentPoseRequest :md5sum-) "7d90ac1bbfd6c3d94c5e9258853f72a9")
(setf (get dsr_msgs::GetCurrentPoseRequest :datatype-) "dsr_msgs/GetCurrentPoseRequest")
(setf (get dsr_msgs::GetCurrentPoseRequest :definition-)
      "#____________________________________________________________________________________________
# get_current_pose
#____________________________________________________________________________________________

int8 space_type # 0=ROBOT_SPACE_JOINT, 1=ROBOT_SPACE_TASK
---
float64[6] pos
bool       success
")

(setf (get dsr_msgs::GetCurrentPoseResponse :md5sum-) "7d90ac1bbfd6c3d94c5e9258853f72a9")
(setf (get dsr_msgs::GetCurrentPoseResponse :datatype-) "dsr_msgs/GetCurrentPoseResponse")
(setf (get dsr_msgs::GetCurrentPoseResponse :definition-)
      "#____________________________________________________________________________________________
# get_current_pose
#____________________________________________________________________________________________

int8 space_type # 0=ROBOT_SPACE_JOINT, 1=ROBOT_SPACE_TASK
---
float64[6] pos
bool       success
")



(provide :dsr_msgs/GetCurrentPose "7d90ac1bbfd6c3d94c5e9258853f72a9")


