;; Auto-generated. Do not edit!


(when (boundp 'hebiros::ModelFkSrv)
  (if (not (find-package "HEBIROS"))
    (make-package "HEBIROS"))
  (shadow 'ModelFkSrv (find-package "HEBIROS")))
(unless (find-package "HEBIROS::MODELFKSRV")
  (make-package "HEBIROS::MODELFKSRV"))
(unless (find-package "HEBIROS::MODELFKSRVREQUEST")
  (make-package "HEBIROS::MODELFKSRVREQUEST"))
(unless (find-package "HEBIROS::MODELFKSRVRESPONSE")
  (make-package "HEBIROS::MODELFKSRVRESPONSE"))

(in-package "ROS")





(intern "*FRAMETYPECENTEROFMASS*" (find-package "HEBIROS::MODELFKSRVREQUEST"))
(shadow '*FRAMETYPECENTEROFMASS* (find-package "HEBIROS::MODELFKSRVREQUEST"))
(defconstant hebiros::ModelFkSrvRequest::*FRAMETYPECENTEROFMASS* 0)
(intern "*FRAMETYPEOUTPUT*" (find-package "HEBIROS::MODELFKSRVREQUEST"))
(shadow '*FRAMETYPEOUTPUT* (find-package "HEBIROS::MODELFKSRVREQUEST"))
(defconstant hebiros::ModelFkSrvRequest::*FRAMETYPEOUTPUT* 1)
(defclass hebiros::ModelFkSrvRequest
  :super ros::object
  :slots (_positions _frame_type ))

(defmethod hebiros::ModelFkSrvRequest
  (:init
   (&key
    ((:positions __positions) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:frame_type __frame_type) 0)
    )
   (send-super :init)
   (setq _positions __positions)
   (setq _frame_type (round __frame_type))
   self)
  (:positions
   (&optional __positions)
   (if __positions (setq _positions __positions)) _positions)
  (:frame_type
   (&optional __frame_type)
   (if __frame_type (setq _frame_type __frame_type)) _frame_type)
  (:serialization-length
   ()
   (+
    ;; float64[] _positions
    (* 8    (length _positions)) 4
    ;; int32 _frame_type
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _positions
     (write-long (length _positions) s)
     (dotimes (i (length _positions))
       (sys::poke (elt _positions i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32 _frame_type
       (write-long _frame_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _positions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positions (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _positions i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int32 _frame_type
     (setq _frame_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass hebiros::ModelFkSrvResponse
  :super ros::object
  :slots (_frames ))

(defmethod hebiros::ModelFkSrvResponse
  (:init
   (&key
    ((:frames __frames) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _frames __frames)
   self)
  (:frames
   (&optional __frames)
   (if __frames (setq _frames __frames)) _frames)
  (:serialization-length
   ()
   (+
    ;; float64[] _frames
    (* 8    (length _frames)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _frames
     (write-long (length _frames) s)
     (dotimes (i (length _frames))
       (sys::poke (elt _frames i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _frames
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _frames (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _frames i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass hebiros::ModelFkSrv
  :super ros::object
  :slots ())

(setf (get hebiros::ModelFkSrv :md5sum-) "0a7e86c595699ff4b039262ca7d90490")
(setf (get hebiros::ModelFkSrv :datatype-) "hebiros/ModelFkSrv")
(setf (get hebiros::ModelFkSrv :request) hebiros::ModelFkSrvRequest)
(setf (get hebiros::ModelFkSrv :response) hebiros::ModelFkSrvResponse)

(defmethod hebiros::ModelFkSrvRequest
  (:response () (instance hebiros::ModelFkSrvResponse :init)))

(setf (get hebiros::ModelFkSrvRequest :md5sum-) "0a7e86c595699ff4b039262ca7d90490")
(setf (get hebiros::ModelFkSrvRequest :datatype-) "hebiros/ModelFkSrvRequest")
(setf (get hebiros::ModelFkSrvRequest :definition-)
      "int32 FrameTypeCenterOfMass = 0
int32 FrameTypeOutput = 1
float64[] positions
int32 frame_type
---
float64[] frames


")

(setf (get hebiros::ModelFkSrvResponse :md5sum-) "0a7e86c595699ff4b039262ca7d90490")
(setf (get hebiros::ModelFkSrvResponse :datatype-) "hebiros/ModelFkSrvResponse")
(setf (get hebiros::ModelFkSrvResponse :definition-)
      "int32 FrameTypeCenterOfMass = 0
int32 FrameTypeOutput = 1
float64[] positions
int32 frame_type
---
float64[] frames


")



(provide :hebiros/ModelFkSrv "0a7e86c595699ff4b039262ca7d90490")


