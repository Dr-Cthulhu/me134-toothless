;; Auto-generated. Do not edit!


(when (boundp 'hebiros::AddModelFromURDFSrv)
  (if (not (find-package "HEBIROS"))
    (make-package "HEBIROS"))
  (shadow 'AddModelFromURDFSrv (find-package "HEBIROS")))
(unless (find-package "HEBIROS::ADDMODELFROMURDFSRV")
  (make-package "HEBIROS::ADDMODELFROMURDFSRV"))
(unless (find-package "HEBIROS::ADDMODELFROMURDFSRVREQUEST")
  (make-package "HEBIROS::ADDMODELFROMURDFSRVREQUEST"))
(unless (find-package "HEBIROS::ADDMODELFROMURDFSRVRESPONSE")
  (make-package "HEBIROS::ADDMODELFROMURDFSRVRESPONSE"))

(in-package "ROS")





(defclass hebiros::AddModelFromURDFSrvRequest
  :super ros::object
  :slots (_model_name _description_param ))

(defmethod hebiros::AddModelFromURDFSrvRequest
  (:init
   (&key
    ((:model_name __model_name) "")
    ((:description_param __description_param) "")
    )
   (send-super :init)
   (setq _model_name (string __model_name))
   (setq _description_param (string __description_param))
   self)
  (:model_name
   (&optional __model_name)
   (if __model_name (setq _model_name __model_name)) _model_name)
  (:description_param
   (&optional __description_param)
   (if __description_param (setq _description_param __description_param)) _description_param)
  (:serialization-length
   ()
   (+
    ;; string _model_name
    4 (length _model_name)
    ;; string _description_param
    4 (length _description_param)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _model_name
       (write-long (length _model_name) s) (princ _model_name s)
     ;; string _description_param
       (write-long (length _description_param) s) (princ _description_param s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _model_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _description_param
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _description_param (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass hebiros::AddModelFromURDFSrvResponse
  :super ros::object
  :slots ())

(defmethod hebiros::AddModelFromURDFSrvResponse
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

(defclass hebiros::AddModelFromURDFSrv
  :super ros::object
  :slots ())

(setf (get hebiros::AddModelFromURDFSrv :md5sum-) "7503667ca5355c5a12e4de26d45dcf94")
(setf (get hebiros::AddModelFromURDFSrv :datatype-) "hebiros/AddModelFromURDFSrv")
(setf (get hebiros::AddModelFromURDFSrv :request) hebiros::AddModelFromURDFSrvRequest)
(setf (get hebiros::AddModelFromURDFSrv :response) hebiros::AddModelFromURDFSrvResponse)

(defmethod hebiros::AddModelFromURDFSrvRequest
  (:response () (instance hebiros::AddModelFromURDFSrvResponse :init)))

(setf (get hebiros::AddModelFromURDFSrvRequest :md5sum-) "7503667ca5355c5a12e4de26d45dcf94")
(setf (get hebiros::AddModelFromURDFSrvRequest :datatype-) "hebiros/AddModelFromURDFSrvRequest")
(setf (get hebiros::AddModelFromURDFSrvRequest :definition-)
      "string model_name

string description_param
---

")

(setf (get hebiros::AddModelFromURDFSrvResponse :md5sum-) "7503667ca5355c5a12e4de26d45dcf94")
(setf (get hebiros::AddModelFromURDFSrvResponse :datatype-) "hebiros/AddModelFromURDFSrvResponse")
(setf (get hebiros::AddModelFromURDFSrvResponse :definition-)
      "string model_name

string description_param
---

")



(provide :hebiros/AddModelFromURDFSrv "7503667ca5355c5a12e4de26d45dcf94")


