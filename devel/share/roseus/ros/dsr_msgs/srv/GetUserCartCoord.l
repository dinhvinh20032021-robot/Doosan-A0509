;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::GetUserCartCoord)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'GetUserCartCoord (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::GETUSERCARTCOORD")
  (make-package "DSR_MSGS::GETUSERCARTCOORD"))
(unless (find-package "DSR_MSGS::GETUSERCARTCOORDREQUEST")
  (make-package "DSR_MSGS::GETUSERCARTCOORDREQUEST"))
(unless (find-package "DSR_MSGS::GETUSERCARTCOORDRESPONSE")
  (make-package "DSR_MSGS::GETUSERCARTCOORDRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::GetUserCartCoordRequest
  :super ros::object
  :slots (_id ))

(defmethod dsr_msgs::GetUserCartCoordRequest
  (:init
   (&key
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; int8 _id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _id
       (write-byte _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _id 127) (setq _id (- _id 256)))
   ;;
   self)
  )

(defclass dsr_msgs::GetUserCartCoordResponse
  :super ros::object
  :slots (_conv_posx _ref _success ))

(defmethod dsr_msgs::GetUserCartCoordResponse
  (:init
   (&key
    ((:conv_posx __conv_posx) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:ref __ref) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _conv_posx __conv_posx)
   (setq _ref (round __ref))
   (setq _success __success)
   self)
  (:conv_posx
   (&optional __conv_posx)
   (if __conv_posx (setq _conv_posx __conv_posx)) _conv_posx)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float64[6] _conv_posx
    (* 8    6)
    ;; int8 _ref
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[6] _conv_posx
     (dotimes (i 6)
       (sys::poke (elt _conv_posx i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8 _ref
       (write-byte _ref s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[6] _conv_posx
   (dotimes (i (length _conv_posx))
     (setf (elt _conv_posx i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int8 _ref
     (setq _ref (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ref 127) (setq _ref (- _ref 256)))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dsr_msgs::GetUserCartCoord
  :super ros::object
  :slots ())

(setf (get dsr_msgs::GetUserCartCoord :md5sum-) "a933e25ab5287ae9b6e7ebd1be20a647")
(setf (get dsr_msgs::GetUserCartCoord :datatype-) "dsr_msgs/GetUserCartCoord")
(setf (get dsr_msgs::GetUserCartCoord :request) dsr_msgs::GetUserCartCoordRequest)
(setf (get dsr_msgs::GetUserCartCoord :response) dsr_msgs::GetUserCartCoordResponse)

(defmethod dsr_msgs::GetUserCartCoordRequest
  (:response () (instance dsr_msgs::GetUserCartCoordResponse :init)))

(setf (get dsr_msgs::GetUserCartCoordRequest :md5sum-) "a933e25ab5287ae9b6e7ebd1be20a647")
(setf (get dsr_msgs::GetUserCartCoordRequest :datatype-) "dsr_msgs/GetUserCartCoordRequest")
(setf (get dsr_msgs::GetUserCartCoordRequest :definition-)
      "#____________________________________________________________________________________________
# posx, ref = get_user_cart_coord(id)   
#____________________________________________________________________________________________
# This service is only available in M2.50 or later

int8       id                # ID of user coord 
---
float64[6] conv_posx         # task pos(posx)  
int8       ref               # Reference coordinate of the coordinate to get
bool       success

")

(setf (get dsr_msgs::GetUserCartCoordResponse :md5sum-) "a933e25ab5287ae9b6e7ebd1be20a647")
(setf (get dsr_msgs::GetUserCartCoordResponse :datatype-) "dsr_msgs/GetUserCartCoordResponse")
(setf (get dsr_msgs::GetUserCartCoordResponse :definition-)
      "#____________________________________________________________________________________________
# posx, ref = get_user_cart_coord(id)   
#____________________________________________________________________________________________
# This service is only available in M2.50 or later

int8       id                # ID of user coord 
---
float64[6] conv_posx         # task pos(posx)  
int8       ref               # Reference coordinate of the coordinate to get
bool       success

")



(provide :dsr_msgs/GetUserCartCoord "a933e25ab5287ae9b6e7ebd1be20a647")


