;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::ConfigCreateTool)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'ConfigCreateTool (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::CONFIGCREATETOOL")
  (make-package "DSR_MSGS::CONFIGCREATETOOL"))
(unless (find-package "DSR_MSGS::CONFIGCREATETOOLREQUEST")
  (make-package "DSR_MSGS::CONFIGCREATETOOLREQUEST"))
(unless (find-package "DSR_MSGS::CONFIGCREATETOOLRESPONSE")
  (make-package "DSR_MSGS::CONFIGCREATETOOLRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::ConfigCreateToolRequest
  :super ros::object
  :slots (_name _weight _cog _inertia ))

(defmethod dsr_msgs::ConfigCreateToolRequest
  (:init
   (&key
    ((:name __name) "")
    ((:weight __weight) 0.0)
    ((:cog __cog) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:inertia __inertia) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _weight (float __weight))
   (setq _cog __cog)
   (setq _inertia __inertia)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:weight
   (&optional __weight)
   (if __weight (setq _weight __weight)) _weight)
  (:cog
   (&optional __cog)
   (if __cog (setq _cog __cog)) _cog)
  (:inertia
   (&optional __inertia)
   (if __inertia (setq _inertia __inertia)) _inertia)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float64 _weight
    8
    ;; float64[3] _cog
    (* 8    3)
    ;; float64[6] _inertia
    (* 8    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float64 _weight
       (sys::poke _weight (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[3] _cog
     (dotimes (i 3)
       (sys::poke (elt _cog i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _inertia
     (dotimes (i 6)
       (sys::poke (elt _inertia i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _weight
     (setq _weight (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[3] _cog
   (dotimes (i (length _cog))
     (setf (elt _cog i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _inertia
   (dotimes (i (length _inertia))
     (setf (elt _inertia i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(defclass dsr_msgs::ConfigCreateToolResponse
  :super ros::object
  :slots (_success ))

(defmethod dsr_msgs::ConfigCreateToolResponse
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

(defclass dsr_msgs::ConfigCreateTool
  :super ros::object
  :slots ())

(setf (get dsr_msgs::ConfigCreateTool :md5sum-) "d748bfa923d2a13eaf013a95f67bd0ec")
(setf (get dsr_msgs::ConfigCreateTool :datatype-) "dsr_msgs/ConfigCreateTool")
(setf (get dsr_msgs::ConfigCreateTool :request) dsr_msgs::ConfigCreateToolRequest)
(setf (get dsr_msgs::ConfigCreateTool :response) dsr_msgs::ConfigCreateToolResponse)

(defmethod dsr_msgs::ConfigCreateToolRequest
  (:response () (instance dsr_msgs::ConfigCreateToolResponse :init)))

(setf (get dsr_msgs::ConfigCreateToolRequest :md5sum-) "d748bfa923d2a13eaf013a95f67bd0ec")
(setf (get dsr_msgs::ConfigCreateToolRequest :datatype-) "dsr_msgs/ConfigCreateToolRequest")
(setf (get dsr_msgs::ConfigCreateToolRequest :definition-)
      "#____________________________________________________________________________________________
# config_create_tool 
# It is a service for registering and using robot Tool information in advance for safety 
#____________________________________________________________________________________________

string          name        # tool name 
float64         weight      # tool weight 
float64[3]      cog         # Center of gravity
float64[6]      inertia     # tool inertia 
---
bool success
")

(setf (get dsr_msgs::ConfigCreateToolResponse :md5sum-) "d748bfa923d2a13eaf013a95f67bd0ec")
(setf (get dsr_msgs::ConfigCreateToolResponse :datatype-) "dsr_msgs/ConfigCreateToolResponse")
(setf (get dsr_msgs::ConfigCreateToolResponse :definition-)
      "#____________________________________________________________________________________________
# config_create_tool 
# It is a service for registering and using robot Tool information in advance for safety 
#____________________________________________________________________________________________

string          name        # tool name 
float64         weight      # tool weight 
float64[3]      cog         # Center of gravity
float64[6]      inertia     # tool inertia 
---
bool success
")



(provide :dsr_msgs/ConfigCreateTool "d748bfa923d2a13eaf013a95f67bd0ec")


