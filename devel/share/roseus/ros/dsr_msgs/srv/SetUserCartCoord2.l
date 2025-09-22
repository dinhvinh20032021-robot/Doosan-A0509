;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::SetUserCartCoord2)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'SetUserCartCoord2 (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::SETUSERCARTCOORD2")
  (make-package "DSR_MSGS::SETUSERCARTCOORD2"))
(unless (find-package "DSR_MSGS::SETUSERCARTCOORD2REQUEST")
  (make-package "DSR_MSGS::SETUSERCARTCOORD2REQUEST"))
(unless (find-package "DSR_MSGS::SETUSERCARTCOORD2RESPONSE")
  (make-package "DSR_MSGS::SETUSERCARTCOORD2RESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::SetUserCartCoord2Request
  :super ros::object
  :slots (_x1 _x2 _x3 _pos _ref ))

(defmethod dsr_msgs::SetUserCartCoord2Request
  (:init
   (&key
    ((:x1 __x1) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:x2 __x2) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:x3 __x3) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:ref __ref) 0)
    )
   (send-super :init)
   (setq _x1 __x1)
   (setq _x2 __x2)
   (setq _x3 __x3)
   (setq _pos __pos)
   (setq _ref (round __ref))
   self)
  (:x1
   (&optional __x1)
   (if __x1 (setq _x1 __x1)) _x1)
  (:x2
   (&optional __x2)
   (if __x2 (setq _x2 __x2)) _x2)
  (:x3
   (&optional __x3)
   (if __x3 (setq _x3 __x3)) _x3)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:serialization-length
   ()
   (+
    ;; float64[6] _x1
    (* 8    6)
    ;; float64[6] _x2
    (* 8    6)
    ;; float64[6] _x3
    (* 8    6)
    ;; float64[6] _pos
    (* 8    6)
    ;; int8 _ref
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _x1
     (dotimes (i 6)
       (sys::poke (elt _x1 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _x2
     (dotimes (i 6)
       (sys::poke (elt _x2 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _x3
     (dotimes (i 6)
       (sys::poke (elt _x3 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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
   ;; float64[6] _x1
   (dotimes (i (length _x1))
     (setf (elt _x1 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _x2
   (dotimes (i (length _x2))
     (setf (elt _x2 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _x3
   (dotimes (i (length _x3))
     (setf (elt _x3 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
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

(defclass dsr_msgs::SetUserCartCoord2Response
  :super ros::object
  :slots (_id _success ))

(defmethod dsr_msgs::SetUserCartCoord2Response
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

(defclass dsr_msgs::SetUserCartCoord2
  :super ros::object
  :slots ())

(setf (get dsr_msgs::SetUserCartCoord2 :md5sum-) "42d11c6c94c1dfc41a37882dbd6da758")
(setf (get dsr_msgs::SetUserCartCoord2 :datatype-) "dsr_msgs/SetUserCartCoord2")
(setf (get dsr_msgs::SetUserCartCoord2 :request) dsr_msgs::SetUserCartCoord2Request)
(setf (get dsr_msgs::SetUserCartCoord2 :response) dsr_msgs::SetUserCartCoord2Response)

(defmethod dsr_msgs::SetUserCartCoord2Request
  (:response () (instance dsr_msgs::SetUserCartCoord2Response :init)))

(setf (get dsr_msgs::SetUserCartCoord2Request :md5sum-) "42d11c6c94c1dfc41a37882dbd6da758")
(setf (get dsr_msgs::SetUserCartCoord2Request :datatype-) "dsr_msgs/SetUserCartCoord2Request")
(setf (get dsr_msgs::SetUserCartCoord2Request :definition-)
      "#____________________________________________________________________________________________
# set_user_cart_coord(x1, x2, x3, pos, ref)
#____________________________________________________________________________________________

float64[6] x1                 # task pos(posx)  
float64[6] x2                 # task pos(posx)  
float64[6] x3                 # task pos(posx)
float64[6] pos                # pos(posx)
int8       ref                # DR_BASE(0), DR_WORLD(2)
                              # <ref is only available in M2.40 or later> 
---
int8    id                    # set user coord (101~200) or fail(-1) 
bool        success   
")

(setf (get dsr_msgs::SetUserCartCoord2Response :md5sum-) "42d11c6c94c1dfc41a37882dbd6da758")
(setf (get dsr_msgs::SetUserCartCoord2Response :datatype-) "dsr_msgs/SetUserCartCoord2Response")
(setf (get dsr_msgs::SetUserCartCoord2Response :definition-)
      "#____________________________________________________________________________________________
# set_user_cart_coord(x1, x2, x3, pos, ref)
#____________________________________________________________________________________________

float64[6] x1                 # task pos(posx)  
float64[6] x2                 # task pos(posx)  
float64[6] x3                 # task pos(posx)
float64[6] pos                # pos(posx)
int8       ref                # DR_BASE(0), DR_WORLD(2)
                              # <ref is only available in M2.40 or later> 
---
int8    id                    # set user coord (101~200) or fail(-1) 
bool        success   
")



(provide :dsr_msgs/SetUserCartCoord2 "42d11c6c94c1dfc41a37882dbd6da758")


