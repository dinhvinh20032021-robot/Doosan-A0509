;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetOrientationError)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetOrientationError (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETORIENTATIONERROR")
  (make-package "DSR_MSGS::GETORIENTATIONERROR"))
(unless (find-package "DSR_MSGS::GETORIENTATIONERRORREQUEST")
  (make-package "DSR_MSGS::GETORIENTATIONERRORREQUEST"))
(unless (find-package "DSR_MSGS::GETORIENTATIONERRORRESPONSE")
  (make-package "DSR_MSGS::GETORIENTATIONERRORRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetOrientationErrorRequest
  :super ros::object
  :slots (_xd _xc _axis ))

(defmethod dsr_msgs::GetOrientationErrorRequest
  (:init
   (&key
    ((:xd __xd) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:xc __xc) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:axis __axis) 0)
    )
   (send-super :init)
   (setq _xd __xd)
   (setq _xc __xc)
   (setq _axis (round __axis))
   self)
  (:xd
   (&optional __xd)
   (if __xd (setq _xd __xd)) _xd)
  (:xc
   (&optional __xc)
   (if __xc (setq _xc __xc)) _xc)
  (:axis
   (&optional __axis)
   (if __axis (setq _axis __axis)) _axis)
  (:serialization-length
   ()
   (+
    ;; float64[6] _xd
    (* 8    6)
    ;; float64[6] _xc
    (* 8    6)
    ;; int8 _axis
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _xd
     (dotimes (i 6)
       (sys::poke (elt _xd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _xc
     (dotimes (i 6)
       (sys::poke (elt _xc i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8 _axis
       (write-byte _axis s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _xd
   (dotimes (i (length _xd))
     (setf (elt _xd i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _xc
   (dotimes (i (length _xc))
     (setf (elt _xc i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int8 _axis
     (setq _axis (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _axis 127) (setq _axis (- _axis 256)))
   ;;
   self)
  )

(defclass dsr_msgs::GetOrientationErrorResponse
  :super ros::object
  :slots (_ori_error _success ))

(defmethod dsr_msgs::GetOrientationErrorResponse
  (:init
   (&key
    ((:ori_error __ori_error) 0.0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _ori_error (float __ori_error))
   (setq _success __success)
   self)
  (:ori_error
   (&optional __ori_error)
   (if __ori_error (setq _ori_error __ori_error)) _ori_error)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float32 _ori_error
    4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _ori_error
       (sys::poke _ori_error (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _ori_error
     (setq _ori_error (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetOrientationError
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetOrientationError :md5sum-) "83afbc83b0df709d838d4a917c3c9c05")
(setf (get dsr_msgs::GetOrientationError :datatype-) "dsr_msgs/GetOrientationError")
(setf (get dsr_msgs::GetOrientationError :request) dsr_msgs::GetOrientationErrorRequest)
(setf (get dsr_msgs::GetOrientationError :response) dsr_msgs::GetOrientationErrorResponse)

(defmethod dsr_msgs::GetOrientationErrorRequest
  (:response () (instance dsr_msgs::GetOrientationErrorResponse :init)))

(setf (get dsr_msgs::GetOrientationErrorRequest :md5sum-) "83afbc83b0df709d838d4a917c3c9c05")
(setf (get dsr_msgs::GetOrientationErrorRequest :datatype-) "dsr_msgs/GetOrientationErrorRequest")
(setf (get dsr_msgs::GetOrientationErrorRequest :definition-)
      "#____________________________________________________________________________________________
# get_orientation_error  
#____________________________________________________________________________________________
# This service returns the orientation error value between the arbitrary poses xd and xc of the axis.

float64[6] xd                # task pos(posx)  
float64[6] xc                # task pos(posx)  
int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
---
float32    ori_error         # orientation error  
bool       success
")

(setf (get dsr_msgs::GetOrientationErrorResponse :md5sum-) "83afbc83b0df709d838d4a917c3c9c05")
(setf (get dsr_msgs::GetOrientationErrorResponse :datatype-) "dsr_msgs/GetOrientationErrorResponse")
(setf (get dsr_msgs::GetOrientationErrorResponse :definition-)
      "#____________________________________________________________________________________________
# get_orientation_error  
#____________________________________________________________________________________________
# This service returns the orientation error value between the arbitrary poses xd and xc of the axis.

float64[6] xd                # task pos(posx)  
float64[6] xc                # task pos(posx)  
int8       axis              # DR_AXIS_X(0), DR_AXIS_Y(1), DR_AXIS_Z(2) 
---
float32    ori_error         # orientation error  
bool       success
")



(provide :dsr_msgs/GetOrientationError "83afbc83b0df709d838d4a917c3c9c05")


