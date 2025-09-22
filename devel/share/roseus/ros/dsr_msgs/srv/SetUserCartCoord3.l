;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetUserCartCoord3)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetUserCartCoord3 (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETUSERCARTCOORD3")
  (make-package "DSR_MSGS::SETUSERCARTCOORD3"))
(unless (find-package "DSR_MSGS::SETUSERCARTCOORD3REQUEST")
  (make-package "DSR_MSGS::SETUSERCARTCOORD3REQUEST"))
(unless (find-package "DSR_MSGS::SETUSERCARTCOORD3RESPONSE")
  (make-package "DSR_MSGS::SETUSERCARTCOORD3RESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetUserCartCoord3Request
  :super ros::object
  :slots (_u1 _v1 _pos _ref ))

(defmethod dsr_msgs::SetUserCartCoord3Request
  (:init
   (&key
    ((:u1 __u1) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:v1 __v1) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:ref __ref) 0)
    )
   (send-super :init)
   (setq _u1 __u1)
   (setq _v1 __v1)
   (setq _pos __pos)
   (setq _ref (round __ref))
   self)
  (:u1
   (&optional __u1)
   (if __u1 (setq _u1 __u1)) _u1)
  (:v1
   (&optional __v1)
   (if __v1 (setq _v1 __v1)) _v1)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:serialization-length
   ()
   (+
    ;; float64[3] _u1
    (* 8    3)
    ;; float64[3] _v1
    (* 8    3)
    ;; float64[6] _pos
    (* 8    6)
    ;; int8 _ref
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[3] _u1
     (dotimes (i 3)
       (sys::poke (elt _u1 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _v1
     (dotimes (i 3)
       (sys::poke (elt _v1 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
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
   ;; float64[3] _u1
   (dotimes (i (length _u1))
     (setf (elt _u1 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _v1
   (dotimes (i (length _v1))
     (setf (elt _v1 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
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

(defclass dsr_msgs::SetUserCartCoord3Response
  :super ros::object
  :slots (_id _success ))

(defmethod dsr_msgs::SetUserCartCoord3Response
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

(defclass dsr_msgs::SetUserCartCoord3
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetUserCartCoord3 :md5sum-) "c6fddb22deca3c301bea04b53df0d0a5")
(setf (get dsr_msgs::SetUserCartCoord3 :datatype-) "dsr_msgs/SetUserCartCoord3")
(setf (get dsr_msgs::SetUserCartCoord3 :request) dsr_msgs::SetUserCartCoord3Request)
(setf (get dsr_msgs::SetUserCartCoord3 :response) dsr_msgs::SetUserCartCoord3Response)

(defmethod dsr_msgs::SetUserCartCoord3Request
  (:response () (instance dsr_msgs::SetUserCartCoord3Response :init)))

(setf (get dsr_msgs::SetUserCartCoord3Request :md5sum-) "c6fddb22deca3c301bea04b53df0d0a5")
(setf (get dsr_msgs::SetUserCartCoord3Request :datatype-) "dsr_msgs/SetUserCartCoord3Request")
(setf (get dsr_msgs::SetUserCartCoord3Request :definition-)
      "#____________________________________________________________________________________________
# set_user_cart_coord(u1, v1, pos, ref) 
#____________________________________________________________________________________________

float64[3] u1                # X-axis unit vector  
float64[3] v1                # Y-axis unit vector 
float64[6] pos               # task pos(posx) 
int8       ref               # DR_BASE(0), DR_WORLD(2)
                             # <ref is only available in M2.40 or later> 
---
int8    id                   # set user coord (101~120) or fail(-1) 
bool        success   
")

(setf (get dsr_msgs::SetUserCartCoord3Response :md5sum-) "c6fddb22deca3c301bea04b53df0d0a5")
(setf (get dsr_msgs::SetUserCartCoord3Response :datatype-) "dsr_msgs/SetUserCartCoord3Response")
(setf (get dsr_msgs::SetUserCartCoord3Response :definition-)
      "#____________________________________________________________________________________________
# set_user_cart_coord(u1, v1, pos, ref) 
#____________________________________________________________________________________________

float64[3] u1                # X-axis unit vector  
float64[3] v1                # Y-axis unit vector 
float64[6] pos               # task pos(posx) 
int8       ref               # DR_BASE(0), DR_WORLD(2)
                             # <ref is only available in M2.40 or later> 
---
int8    id                   # set user coord (101~120) or fail(-1) 
bool        success   
")



(provide :dsr_msgs/SetUserCartCoord3 "c6fddb22deca3c301bea04b53df0d0a5")


