;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetUserCartCoord1)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetUserCartCoord1 (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETUSERCARTCOORD1")
  (make-package "DSR_MSGS::SETUSERCARTCOORD1"))
(unless (find-package "DSR_MSGS::SETUSERCARTCOORD1REQUEST")
  (make-package "DSR_MSGS::SETUSERCARTCOORD1REQUEST"))
(unless (find-package "DSR_MSGS::SETUSERCARTCOORD1RESPONSE")
  (make-package "DSR_MSGS::SETUSERCARTCOORD1RESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetUserCartCoord1Request
  :super ros::object
  :slots (_pos _ref ))

(defmethod dsr_msgs::SetUserCartCoord1Request
  (:init
   (&key
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:ref __ref) 0)
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _ref (round __ref))
   self)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:serialization-length
   ()
   (+
    ;; float64[6] _pos
    (* 8    6)
    ;; int8 _ref
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
     ;; int8 _ref
       (write-byte _ref s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;;
   self)
  )

(defclass dsr_msgs::SetUserCartCoord1Response
  :super ros::object
  :slots (_id _success ))

(defmethod dsr_msgs::SetUserCartCoord1Response
  (:init
   (&key
    ((:id __id) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _success __success)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int8 _id
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _id
       (write-byte _id s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _id 127) (setq _id (- _id 256)))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::SetUserCartCoord1
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetUserCartCoord1 :md5sum-) "42b1d22dac70724bd7473cd4cc41f2c4")
(setf (get dsr_msgs::SetUserCartCoord1 :datatype-) "dsr_msgs/SetUserCartCoord1")
(setf (get dsr_msgs::SetUserCartCoord1 :request) dsr_msgs::SetUserCartCoord1Request)
(setf (get dsr_msgs::SetUserCartCoord1 :response) dsr_msgs::SetUserCartCoord1Response)

(defmethod dsr_msgs::SetUserCartCoord1Request
  (:response () (instance dsr_msgs::SetUserCartCoord1Response :init)))

(setf (get dsr_msgs::SetUserCartCoord1Request :md5sum-) "42b1d22dac70724bd7473cd4cc41f2c4")
(setf (get dsr_msgs::SetUserCartCoord1Request :datatype-) "dsr_msgs/SetUserCartCoord1Request")
(setf (get dsr_msgs::SetUserCartCoord1Request :definition-)
      "#____________________________________________________________________________________________
# set_user_cart_coord(pos, ref)   
#____________________________________________________________________________________________

float64[6] pos                # task pos(posx)  
int8       ref                # DR_BASE(0), DR_WORLD(2)
                              # <ref is only available in M2.40 or later> 
---
int8    id                    # set user coord (101~120) or fail(-1)
bool        success    
")

(setf (get dsr_msgs::SetUserCartCoord1Response :md5sum-) "42b1d22dac70724bd7473cd4cc41f2c4")
(setf (get dsr_msgs::SetUserCartCoord1Response :datatype-) "dsr_msgs/SetUserCartCoord1Response")
(setf (get dsr_msgs::SetUserCartCoord1Response :definition-)
      "#____________________________________________________________________________________________
# set_user_cart_coord(pos, ref)   
#____________________________________________________________________________________________

float64[6] pos                # task pos(posx)  
int8       ref                # DR_BASE(0), DR_WORLD(2)
                              # <ref is only available in M2.40 or later> 
---
int8    id                    # set user coord (101~120) or fail(-1)
bool        success    
")



(provide :dsr_msgs/SetUserCartCoord1 "42b1d22dac70724bd7473cd4cc41f2c4")


