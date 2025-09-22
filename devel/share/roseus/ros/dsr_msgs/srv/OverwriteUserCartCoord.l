;; Auto-generated. Do not edit!


(when (boundp 'dsr_msgs::OverwriteUserCartCoord)
  (if (not (find-package "DSR_MSGS"))
    (make-package "DSR_MSGS"))
  (shadow 'OverwriteUserCartCoord (find-package "DSR_MSGS")))
(unless (find-package "DSR_MSGS::OVERWRITEUSERCARTCOORD")
  (make-package "DSR_MSGS::OVERWRITEUSERCARTCOORD"))
(unless (find-package "DSR_MSGS::OVERWRITEUSERCARTCOORDREQUEST")
  (make-package "DSR_MSGS::OVERWRITEUSERCARTCOORDREQUEST"))
(unless (find-package "DSR_MSGS::OVERWRITEUSERCARTCOORDRESPONSE")
  (make-package "DSR_MSGS::OVERWRITEUSERCARTCOORDRESPONSE"))

(in-package "ROS")





(defclass dsr_msgs::OverwriteUserCartCoordRequest
  :super ros::object
  :slots (_id _pos _ref ))

(defmethod dsr_msgs::OverwriteUserCartCoordRequest
  (:init
   (&key
    ((:id __id) 0)
    ((:pos __pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:ref __ref) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _pos __pos)
   (setq _ref (round __ref))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:ref
   (&optional __ref)
   (if __ref (setq _ref __ref)) _ref)
  (:serialization-length
   ()
   (+
    ;; int8 _id
    1
    ;; float64[6] _pos
    (* 8    6)
    ;; int8 _ref
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _id
       (write-byte _id s)
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
   ;; int8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _id 127) (setq _id (- _id 256)))
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

(defclass dsr_msgs::OverwriteUserCartCoordResponse
  :super ros::object
  :slots (_id _success ))

(defmethod dsr_msgs::OverwriteUserCartCoordResponse
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

(defclass dsr_msgs::OverwriteUserCartCoord
  :super ros::object
  :slots ())

(setf (get dsr_msgs::OverwriteUserCartCoord :md5sum-) "dd70ea2a3d7fe4aee2fb401824577ffd")
(setf (get dsr_msgs::OverwriteUserCartCoord :datatype-) "dsr_msgs/OverwriteUserCartCoord")
(setf (get dsr_msgs::OverwriteUserCartCoord :request) dsr_msgs::OverwriteUserCartCoordRequest)
(setf (get dsr_msgs::OverwriteUserCartCoord :response) dsr_msgs::OverwriteUserCartCoordResponse)

(defmethod dsr_msgs::OverwriteUserCartCoordRequest
  (:response () (instance dsr_msgs::OverwriteUserCartCoordResponse :init)))

(setf (get dsr_msgs::OverwriteUserCartCoordRequest :md5sum-) "dd70ea2a3d7fe4aee2fb401824577ffd")
(setf (get dsr_msgs::OverwriteUserCartCoordRequest :datatype-) "dsr_msgs/OverwriteUserCartCoordRequest")
(setf (get dsr_msgs::OverwriteUserCartCoordRequest :definition-)
      "#____________________________________________________________________________________________
# overwrite_user_cart_coord   
#____________________________________________________________________________________________
# This service is only available in M2.50 or later

int8       id                # ID of user coord 
float64[6] pos               # task pos(posx)  
int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2)
---
int8       id                # Successful coordinate setting, Set user coordinate ID (101 - 200)
                             # (-1) Failed coordinate setting
bool       success                             
")

(setf (get dsr_msgs::OverwriteUserCartCoordResponse :md5sum-) "dd70ea2a3d7fe4aee2fb401824577ffd")
(setf (get dsr_msgs::OverwriteUserCartCoordResponse :datatype-) "dsr_msgs/OverwriteUserCartCoordResponse")
(setf (get dsr_msgs::OverwriteUserCartCoordResponse :definition-)
      "#____________________________________________________________________________________________
# overwrite_user_cart_coord   
#____________________________________________________________________________________________
# This service is only available in M2.50 or later

int8       id                # ID of user coord 
float64[6] pos               # task pos(posx)  
int8       ref        #= 0   # DR_BASE(0), DR_WORLD(2)
---
int8       id                # Successful coordinate setting, Set user coordinate ID (101 - 200)
                             # (-1) Failed coordinate setting
bool       success                             
")



(provide :dsr_msgs/OverwriteUserCartCoord "dd70ea2a3d7fe4aee2fb401824577ffd")


