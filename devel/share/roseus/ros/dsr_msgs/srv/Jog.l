;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::Jog)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'Jog (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::JOG")
  (make-package "DSR_MSGS::JOG"))
(unless (find-package "DSR_MSGS::JOGREQUEST")
  (make-package "DSR_MSGS::JOGREQUEST"))
(unless (find-package "DSR_MSGS::JOGRESPONSE")
  (make-package "DSR_MSGS::JOGRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::JogRequest
  :super ros::object
  :slots (_jog_axis _move_reference _speed ))

(defmethod dsr_msgs::JogRequest
  (:init
   (&key
    ((:jog_axis __jog_axis) 0)
    ((:move_reference __move_reference) 0)
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _jog_axis (round __jog_axis))
   (setq _move_reference (round __move_reference))
   (setq _speed (float __speed))
   self)
  (:jog_axis
   (&optional __jog_axis)
   (if __jog_axis (setq _jog_axis __jog_axis)) _jog_axis)
  (:move_reference
   (&optional __move_reference)
   (if __move_reference (setq _move_reference __move_reference)) _move_reference)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; int8 _jog_axis
    1
    ;; int8 _move_reference
    1
    ;; float64 _speed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _jog_axis
       (write-byte _jog_axis s)
     ;; int8 _move_reference
       (write-byte _move_reference s)
     ;; float64 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _jog_axis
     (setq _jog_axis (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _jog_axis 127) (setq _jog_axis (- _jog_axis 256)))
   ;; int8 _move_reference
     (setq _move_reference (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _move_reference 127) (setq _move_reference (- _move_reference 256)))
   ;; float64 _speed
     (setq _speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass dsr_msgs::JogResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::JogResponse
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

(defclass dsr_msgs::Jog
  :super ros::object
  :slots ())

(setf (get dsr_msgs::Jog :md5sum-) "240aba792f78029c4074628f5a2713fe")
(setf (get dsr_msgs::Jog :datatype-) "dsr_msgs/Jog")
(setf (get dsr_msgs::Jog :request) dsr_msgs::JogRequest)
(setf (get dsr_msgs::Jog :response) dsr_msgs::JogResponse)

(defmethod dsr_msgs::JogRequest
  (:response () (instance dsr_msgs::JogResponse :init)))

(setf (get dsr_msgs::JogRequest :md5sum-) "240aba792f78029c4074628f5a2713fe")
(setf (get dsr_msgs::JogRequest :datatype-) "dsr_msgs/JogRequest")
(setf (get dsr_msgs::JogRequest :definition-)
      "#____________________________________________________________________________________________
# single jog
# single jog speed = (250mm/s) x speed [%] 
#____________________________________________________________________________________________

int8 jog_axis          # 0 ~ 5 : JOINT 1 ~ 6 
                       # 6 ~ 11: TASK 1 ~ 6 (X,Y,Z,rx,ry,rz)
int8 move_reference    # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL
float64 speed          # jog speed [%] : + forward , 0=stop, - backward  
---
bool success

")

(setf (get dsr_msgs::JogResponse :md5sum-) "240aba792f78029c4074628f5a2713fe")
(setf (get dsr_msgs::JogResponse :datatype-) "dsr_msgs/JogResponse")
(setf (get dsr_msgs::JogResponse :definition-)
      "#____________________________________________________________________________________________
# single jog
# single jog speed = (250mm/s) x speed [%] 
#____________________________________________________________________________________________

int8 jog_axis          # 0 ~ 5 : JOINT 1 ~ 6 
                       # 6 ~ 11: TASK 1 ~ 6 (X,Y,Z,rx,ry,rz)
int8 move_reference    # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL
float64 speed          # jog speed [%] : + forward , 0=stop, - backward  
---
bool success

")



(provide :dsr_msgs/Jog "240aba792f78029c4074628f5a2713fe")


