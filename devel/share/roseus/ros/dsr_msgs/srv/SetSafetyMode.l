;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetSafetyMode)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetSafetyMode (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETSAFETYMODE")
  (make-package "DSR_MSGS::SETSAFETYMODE"))
(unless (find-package "DSR_MSGS::SETSAFETYMODEREQUEST")
  (make-package "DSR_MSGS::SETSAFETYMODEREQUEST"))
(unless (find-package "DSR_MSGS::SETSAFETYMODERESPONSE")
  (make-package "DSR_MSGS::SETSAFETYMODERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetSafetyModeRequest
  :super ros::object
  :slots (_safety_mode _safety_mode_event ))

(defmethod dsr_msgs::SetSafetyModeRequest
  (:init
   (&key
    ((:safety_mode __safety_mode) 0)
    ((:safety_mode_event __safety_mode_event) 0)
    )
   (send-super :init)
   (setq _safety_mode (round __safety_mode))
   (setq _safety_mode_event (round __safety_mode_event))
   self)
  (:safety_mode
   (&optional __safety_mode)
   (if __safety_mode (setq _safety_mode __safety_mode)) _safety_mode)
  (:safety_mode_event
   (&optional __safety_mode_event)
   (if __safety_mode_event (setq _safety_mode_event __safety_mode_event)) _safety_mode_event)
  (:serialization-length
   ()
   (+
    ;; int8 _safety_mode
    1
    ;; int8 _safety_mode_event
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _safety_mode
       (write-byte _safety_mode s)
     ;; int8 _safety_mode_event
       (write-byte _safety_mode_event s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _safety_mode
     (setq _safety_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _safety_mode 127) (setq _safety_mode (- _safety_mode 256)))
   ;; int8 _safety_mode_event
     (setq _safety_mode_event (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _safety_mode_event 127) (setq _safety_mode_event (- _safety_mode_event 256)))
   ;;
   self)
  )

(defclass dsr_msgs::SetSafetyModeResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::SetSafetyModeResponse
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

(defclass dsr_msgs::SetSafetyMode
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetSafetyMode :md5sum-) "82c2323d83064354d4a93feb6b800608")
(setf (get dsr_msgs::SetSafetyMode :datatype-) "dsr_msgs/SetSafetyMode")
(setf (get dsr_msgs::SetSafetyMode :request) dsr_msgs::SetSafetyModeRequest)
(setf (get dsr_msgs::SetSafetyMode :response) dsr_msgs::SetSafetyModeResponse)

(defmethod dsr_msgs::SetSafetyModeRequest
  (:response () (instance dsr_msgs::SetSafetyModeResponse :init)))

(setf (get dsr_msgs::SetSafetyModeRequest :md5sum-) "82c2323d83064354d4a93feb6b800608")
(setf (get dsr_msgs::SetSafetyModeRequest :datatype-) "dsr_msgs/SetSafetyModeRequest")
(setf (get dsr_msgs::SetSafetyModeRequest :definition-)
      "#____________________________________________________________________________________________
# set_safety_mode
#____________________________________________________________________________________________

int8 safety_mode  
int8 safety_mode_event
---
bool success
")

(setf (get dsr_msgs::SetSafetyModeResponse :md5sum-) "82c2323d83064354d4a93feb6b800608")
(setf (get dsr_msgs::SetSafetyModeResponse :datatype-) "dsr_msgs/SetSafetyModeResponse")
(setf (get dsr_msgs::SetSafetyModeResponse :definition-)
      "#____________________________________________________________________________________________
# set_safety_mode
#____________________________________________________________________________________________

int8 safety_mode  
int8 safety_mode_event
---
bool success
")



(provide :dsr_msgs/SetSafetyMode "82c2323d83064354d4a93feb6b800608")


