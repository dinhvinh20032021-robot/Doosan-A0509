;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ReleaseProtectiveStop)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ReleaseProtectiveStop (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::RELEASEPROTECTIVESTOP")
  (make-package "DSR_MSGS::RELEASEPROTECTIVESTOP"))
(unless (find-package "DSR_MSGS::RELEASEPROTECTIVESTOPREQUEST")
  (make-package "DSR_MSGS::RELEASEPROTECTIVESTOPREQUEST"))
(unless (find-package "DSR_MSGS::RELEASEPROTECTIVESTOPRESPONSE")
  (make-package "DSR_MSGS::RELEASEPROTECTIVESTOPRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::ReleaseProtectiveStopRequest
  :super ros::object
  :slots (_release_mode ))

(defmethod dsr_msgs::ReleaseProtectiveStopRequest
  (:init
   (&key
    ((:release_mode __release_mode) 0)
    )
   (send-super :init)
   (setq _release_mode (round __release_mode))
   self)
  (:release_mode
   (&optional __release_mode)
   (if __release_mode (setq _release_mode __release_mode)) _release_mode)
  (:serialization-length
   ()
   (+
    ;; int8 _release_mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _release_mode
       (write-byte _release_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _release_mode
     (setq _release_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _release_mode 127) (setq _release_mode (- _release_mode 256)))
   ;;
   self)
  )

(defclass dsr_msgs::ReleaseProtectiveStopResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::ReleaseProtectiveStopResponse
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

(defclass dsr_msgs::ReleaseProtectiveStop
  :super ros::object
  :slots ())

(setf (get dsr_msgs::ReleaseProtectiveStop :md5sum-) "3cf8efd6d59d99783a077dda000b1bbb")
(setf (get dsr_msgs::ReleaseProtectiveStop :datatype-) "dsr_msgs/ReleaseProtectiveStop")
(setf (get dsr_msgs::ReleaseProtectiveStop :request) dsr_msgs::ReleaseProtectiveStopRequest)
(setf (get dsr_msgs::ReleaseProtectiveStop :response) dsr_msgs::ReleaseProtectiveStopResponse)

(defmethod dsr_msgs::ReleaseProtectiveStopRequest
  (:response () (instance dsr_msgs::ReleaseProtectiveStopResponse :init)))

(setf (get dsr_msgs::ReleaseProtectiveStopRequest :md5sum-) "3cf8efd6d59d99783a077dda000b1bbb")
(setf (get dsr_msgs::ReleaseProtectiveStopRequest :datatype-) "dsr_msgs/ReleaseProtectiveStopRequest")
(setf (get dsr_msgs::ReleaseProtectiveStopRequest :definition-)
      "#____________________________________________________________________________________________
# release_protective_stop
#____________________________________________________________________________________________

int8 release_mode     # 0=RELEASE_MODE_STOP, 1=RELEASE_MODE_RESUME, 2=RELEASE_MODE_RELEASE, 3=RELEASE_MODE_RESET
---
bool success
")

(setf (get dsr_msgs::ReleaseProtectiveStopResponse :md5sum-) "3cf8efd6d59d99783a077dda000b1bbb")
(setf (get dsr_msgs::ReleaseProtectiveStopResponse :datatype-) "dsr_msgs/ReleaseProtectiveStopResponse")
(setf (get dsr_msgs::ReleaseProtectiveStopResponse :definition-)
      "#____________________________________________________________________________________________
# release_protective_stop
#____________________________________________________________________________________________

int8 release_mode     # 0=RELEASE_MODE_STOP, 1=RELEASE_MODE_RESUME, 2=RELEASE_MODE_RELEASE, 3=RELEASE_MODE_RESET
---
bool success
")



(provide :dsr_msgs/ReleaseProtectiveStop "3cf8efd6d59d99783a077dda000b1bbb")


