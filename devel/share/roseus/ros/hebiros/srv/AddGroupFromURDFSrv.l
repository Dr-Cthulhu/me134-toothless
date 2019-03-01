;; Auto-generated. Do not edit!


(when (boundp 'hebiros::AddGroupFromURDFSrv)
  (if (not (find-package "HEBIROS"))
    (make-package "HEBIROS"))
  (shadow 'AddGroupFromURDFSrv (find-package "HEBIROS")))
(unless (find-package "HEBIROS::ADDGROUPFROMURDFSRV")
  (make-package "HEBIROS::ADDGROUPFROMURDFSRV"))
(unless (find-package "HEBIROS::ADDGROUPFROMURDFSRVREQUEST")
  (make-package "HEBIROS::ADDGROUPFROMURDFSRVREQUEST"))
(unless (find-package "HEBIROS::ADDGROUPFROMURDFSRVRESPONSE")
  (make-package "HEBIROS::ADDGROUPFROMURDFSRVRESPONSE"))

(in-package "ROS")





(defclass hebiros::AddGroupFromURDFSrvRequest
  :super ros::object
  :slots (_group_name ))

(defmethod hebiros::AddGroupFromURDFSrvRequest
  (:init
   (&key
    ((:group_name __group_name) "")
    )
   (send-super :init)
   (setq _group_name (string __group_name))
   self)
  (:group_name
   (&optional __group_name)
   (if __group_name (setq _group_name __group_name)) _group_name)
  (:serialization-length
   ()
   (+
    ;; string _group_name
    4 (length _group_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _group_name
       (write-long (length _group_name) s) (princ _group_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _group_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _group_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass hebiros::AddGroupFromURDFSrvResponse
  :super ros::object
  :slots ())

(defmethod hebiros::AddGroupFromURDFSrvResponse
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

(defclass hebiros::AddGroupFromURDFSrv
  :super ros::object
  :slots ())

(setf (get hebiros::AddGroupFromURDFSrv :md5sum-) "967d0b0c0d858ded8a6a69abbce0c981")
(setf (get hebiros::AddGroupFromURDFSrv :datatype-) "hebiros/AddGroupFromURDFSrv")
(setf (get hebiros::AddGroupFromURDFSrv :request) hebiros::AddGroupFromURDFSrvRequest)
(setf (get hebiros::AddGroupFromURDFSrv :response) hebiros::AddGroupFromURDFSrvResponse)

(defmethod hebiros::AddGroupFromURDFSrvRequest
  (:response () (instance hebiros::AddGroupFromURDFSrvResponse :init)))

(setf (get hebiros::AddGroupFromURDFSrvRequest :md5sum-) "967d0b0c0d858ded8a6a69abbce0c981")
(setf (get hebiros::AddGroupFromURDFSrvRequest :datatype-) "hebiros/AddGroupFromURDFSrvRequest")
(setf (get hebiros::AddGroupFromURDFSrvRequest :definition-)
      "string group_name
---

")

(setf (get hebiros::AddGroupFromURDFSrvResponse :md5sum-) "967d0b0c0d858ded8a6a69abbce0c981")
(setf (get hebiros::AddGroupFromURDFSrvResponse :datatype-) "hebiros/AddGroupFromURDFSrvResponse")
(setf (get hebiros::AddGroupFromURDFSrvResponse :definition-)
      "string group_name
---

")



(provide :hebiros/AddGroupFromURDFSrv "967d0b0c0d858ded8a6a69abbce0c981")


