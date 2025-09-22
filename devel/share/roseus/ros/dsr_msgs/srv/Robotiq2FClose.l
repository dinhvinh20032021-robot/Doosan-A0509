;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::Robotiq2FClose)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'Robotiq2FClose (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::ROBOTIQ2FCLOSE")
  (make-package "DSR_MSGS::ROBOTIQ2FCLOSE"))
(unless (find-package "DSR_MSGS::ROBOTIQ2FCLOSEREQUEST")
  (make-package "DSR_MSGS::ROBOTIQ2FCLOSEREQUEST"))
(unless (find-package "DSR_MSGS::ROBOTIQ2FCLOSERESPONSE")
  (make-package "DSR_MSGS::ROBOTIQ2FCLOSERESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::Robotiq2FCloseRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::Robotiq2FCloseRequest
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

(defclass dsr_msgs::Robotiq2FCloseResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::Robotiq2FCloseResponse
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

(defclass dsr_msgs::Robotiq2FClose
  :super ros::object
  :slots ())

(setf (get dsr_msgs::Robotiq2FClose :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::Robotiq2FClose :datatype-) "dsr_msgs/Robotiq2FClose")
(setf (get dsr_msgs::Robotiq2FClose :request) dsr_msgs::Robotiq2FCloseRequest)
(setf (get dsr_msgs::Robotiq2FClose :response) dsr_msgs::Robotiq2FCloseResponse)

(defmethod dsr_msgs::Robotiq2FCloseRequest
  (:response () (instance dsr_msgs::Robotiq2FCloseResponse :init)))

(setf (get dsr_msgs::Robotiq2FCloseRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::Robotiq2FCloseRequest :datatype-) "dsr_msgs/Robotiq2FCloseRequest")
(setf (get dsr_msgs::Robotiq2FCloseRequest :definition-)
      "#____________________________________________________________________________________________
# robotiq_2f_close
# close the virtual robotiq gripper  
#____________________________________________________________________________________________

# None     
---
bool success
")

(setf (get dsr_msgs::Robotiq2FCloseResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::Robotiq2FCloseResponse :datatype-) "dsr_msgs/Robotiq2FCloseResponse")
(setf (get dsr_msgs::Robotiq2FCloseResponse :definition-)
      "#____________________________________________________________________________________________
# robotiq_2f_close
# close the virtual robotiq gripper  
#____________________________________________________________________________________________

# None     
---
bool success
")



(provide :dsr_msgs/Robotiq2FClose "358e233cde0c8a8bcfea4ce193f8fc15")


