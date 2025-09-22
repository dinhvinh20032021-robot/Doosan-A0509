;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::DrlStart)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'DrlStart (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::DRLSTART")
  (make-package "DSR_MSGS::DRLSTART"))
(unless (find-package "DSR_MSGS::DRLSTARTREQUEST")
  (make-package "DSR_MSGS::DRLSTARTREQUEST"))
(unless (find-package "DSR_MSGS::DRLSTARTRESPONSE")
  (make-package "DSR_MSGS::DRLSTARTRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::DrlStartRequest
  :super ros::object
  :slots (_robotSystem _code ))

(defmethod dsr_msgs::DrlStartRequest
  (:init
   (&key
    ((:robotSystem __robotSystem) 0)
    ((:code __code) "")
    )
   (send-super :init)
   (setq _robotSystem (round __robotSystem))
   (setq _code (string __code))
   self)
  (:robotSystem
   (&optional __robotSystem)
   (if __robotSystem (setq _robotSystem __robotSystem)) _robotSystem)
  (:code
   (&optional __code)
   (if __code (setq _code __code)) _code)
  (:serialization-length
   ()
   (+
    ;; int8 _robotSystem
    1
    ;; string _code
    4 (length _code)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _robotSystem
       (write-byte _robotSystem s)
     ;; string _code
       (write-long (length _code) s) (princ _code s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _robotSystem
     (setq _robotSystem (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _robotSystem 127) (setq _robotSystem (- _robotSystem 256)))
   ;; string _code
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _code (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass dsr_msgs::DrlStartResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::DrlStartResponse
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

(defclass dsr_msgs::DrlStart
  :super ros::object
  :slots ())

(setf (get dsr_msgs::DrlStart :md5sum-) "141a24be8b7e223a4678cb5eeb926bd6")
(setf (get dsr_msgs::DrlStart :datatype-) "dsr_msgs/DrlStart")
(setf (get dsr_msgs::DrlStart :request) dsr_msgs::DrlStartRequest)
(setf (get dsr_msgs::DrlStart :response) dsr_msgs::DrlStartResponse)

(defmethod dsr_msgs::DrlStartRequest
  (:response () (instance dsr_msgs::DrlStartResponse :init)))

(setf (get dsr_msgs::DrlStartRequest :md5sum-) "141a24be8b7e223a4678cb5eeb926bd6")
(setf (get dsr_msgs::DrlStartRequest :datatype-) "dsr_msgs/DrlStartRequest")
(setf (get dsr_msgs::DrlStartRequest :definition-)
      "#____________________________________________________________________________________________
# drl_script_run  
# This is a service to execute a program configured in the DRL language in the robot controller.
#____________________________________________________________________________________________

int8 robotSystem    # Robot System Mode 0 : Real, 1 : virtual
string  code        # drl code       
---
bool success
")

(setf (get dsr_msgs::DrlStartResponse :md5sum-) "141a24be8b7e223a4678cb5eeb926bd6")
(setf (get dsr_msgs::DrlStartResponse :datatype-) "dsr_msgs/DrlStartResponse")
(setf (get dsr_msgs::DrlStartResponse :definition-)
      "#____________________________________________________________________________________________
# drl_script_run  
# This is a service to execute a program configured in the DRL language in the robot controller.
#____________________________________________________________________________________________

int8 robotSystem    # Robot System Mode 0 : Real, 1 : virtual
string  code        # drl code       
---
bool success
")



(provide :dsr_msgs/DrlStart "141a24be8b7e223a4678cb5eeb926bd6")


