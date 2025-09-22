;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ReleaseComplianceCtrl)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ReleaseComplianceCtrl (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::RELEASECOMPLIANCECTRL")
  (make-package "DSR_MSGS::RELEASECOMPLIANCECTRL"))
(unless (find-package "DSR_MSGS::RELEASECOMPLIANCECTRLREQUEST")
  (make-package "DSR_MSGS::RELEASECOMPLIANCECTRLREQUEST"))
(unless (find-package "DSR_MSGS::RELEASECOMPLIANCECTRLRESPONSE")
  (make-package "DSR_MSGS::RELEASECOMPLIANCECTRLRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::ReleaseComplianceCtrlRequest
  :super ros::object
  :slots ())

(defmethod dsr_msgs::ReleaseComplianceCtrlRequest
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

(defclass dsr_msgs::ReleaseComplianceCtrlResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::ReleaseComplianceCtrlResponse
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

(defclass dsr_msgs::ReleaseComplianceCtrl
  :super ros::object
  :slots ())

(setf (get dsr_msgs::ReleaseComplianceCtrl :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::ReleaseComplianceCtrl :datatype-) "dsr_msgs/ReleaseComplianceCtrl")
(setf (get dsr_msgs::ReleaseComplianceCtrl :request) dsr_msgs::ReleaseComplianceCtrlRequest)
(setf (get dsr_msgs::ReleaseComplianceCtrl :response) dsr_msgs::ReleaseComplianceCtrlResponse)

(defmethod dsr_msgs::ReleaseComplianceCtrlRequest
  (:response () (instance dsr_msgs::ReleaseComplianceCtrlResponse :init)))

(setf (get dsr_msgs::ReleaseComplianceCtrlRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::ReleaseComplianceCtrlRequest :datatype-) "dsr_msgs/ReleaseComplianceCtrlRequest")
(setf (get dsr_msgs::ReleaseComplianceCtrlRequest :definition-)
      "#____________________________________________________________________________________________
# release_compliance_ctrl  
#____________________________________________________________________________________________

---
bool       success 
")

(setf (get dsr_msgs::ReleaseComplianceCtrlResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get dsr_msgs::ReleaseComplianceCtrlResponse :datatype-) "dsr_msgs/ReleaseComplianceCtrlResponse")
(setf (get dsr_msgs::ReleaseComplianceCtrlResponse :definition-)
      "#____________________________________________________________________________________________
# release_compliance_ctrl  
#____________________________________________________________________________________________

---
bool       success 
")



(provide :dsr_msgs/ReleaseComplianceCtrl "358e233cde0c8a8bcfea4ce193f8fc15")


