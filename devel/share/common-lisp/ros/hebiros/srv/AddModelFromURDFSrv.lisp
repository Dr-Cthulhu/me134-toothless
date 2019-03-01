; Auto-generated. Do not edit!


(cl:in-package hebiros-srv)


;//! \htmlinclude AddModelFromURDFSrv-request.msg.html

(cl:defclass <AddModelFromURDFSrv-request> (roslisp-msg-protocol:ros-message)
  ((model_name
    :reader model_name
    :initarg :model_name
    :type cl:string
    :initform "")
   (description_param
    :reader description_param
    :initarg :description_param
    :type cl:string
    :initform ""))
)

(cl:defclass AddModelFromURDFSrv-request (<AddModelFromURDFSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddModelFromURDFSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddModelFromURDFSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebiros-srv:<AddModelFromURDFSrv-request> is deprecated: use hebiros-srv:AddModelFromURDFSrv-request instead.")))

(cl:ensure-generic-function 'model_name-val :lambda-list '(m))
(cl:defmethod model_name-val ((m <AddModelFromURDFSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebiros-srv:model_name-val is deprecated.  Use hebiros-srv:model_name instead.")
  (model_name m))

(cl:ensure-generic-function 'description_param-val :lambda-list '(m))
(cl:defmethod description_param-val ((m <AddModelFromURDFSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebiros-srv:description_param-val is deprecated.  Use hebiros-srv:description_param instead.")
  (description_param m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddModelFromURDFSrv-request>) ostream)
  "Serializes a message object of type '<AddModelFromURDFSrv-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description_param))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description_param))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddModelFromURDFSrv-request>) istream)
  "Deserializes a message object of type '<AddModelFromURDFSrv-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description_param) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description_param) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddModelFromURDFSrv-request>)))
  "Returns string type for a service object of type '<AddModelFromURDFSrv-request>"
  "hebiros/AddModelFromURDFSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddModelFromURDFSrv-request)))
  "Returns string type for a service object of type 'AddModelFromURDFSrv-request"
  "hebiros/AddModelFromURDFSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddModelFromURDFSrv-request>)))
  "Returns md5sum for a message object of type '<AddModelFromURDFSrv-request>"
  "7503667ca5355c5a12e4de26d45dcf94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddModelFromURDFSrv-request)))
  "Returns md5sum for a message object of type 'AddModelFromURDFSrv-request"
  "7503667ca5355c5a12e4de26d45dcf94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddModelFromURDFSrv-request>)))
  "Returns full string definition for message of type '<AddModelFromURDFSrv-request>"
  (cl:format cl:nil "string model_name~%~%string description_param~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddModelFromURDFSrv-request)))
  "Returns full string definition for message of type 'AddModelFromURDFSrv-request"
  (cl:format cl:nil "string model_name~%~%string description_param~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddModelFromURDFSrv-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'model_name))
     4 (cl:length (cl:slot-value msg 'description_param))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddModelFromURDFSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddModelFromURDFSrv-request
    (cl:cons ':model_name (model_name msg))
    (cl:cons ':description_param (description_param msg))
))
;//! \htmlinclude AddModelFromURDFSrv-response.msg.html

(cl:defclass <AddModelFromURDFSrv-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass AddModelFromURDFSrv-response (<AddModelFromURDFSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddModelFromURDFSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddModelFromURDFSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebiros-srv:<AddModelFromURDFSrv-response> is deprecated: use hebiros-srv:AddModelFromURDFSrv-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddModelFromURDFSrv-response>) ostream)
  "Serializes a message object of type '<AddModelFromURDFSrv-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddModelFromURDFSrv-response>) istream)
  "Deserializes a message object of type '<AddModelFromURDFSrv-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddModelFromURDFSrv-response>)))
  "Returns string type for a service object of type '<AddModelFromURDFSrv-response>"
  "hebiros/AddModelFromURDFSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddModelFromURDFSrv-response)))
  "Returns string type for a service object of type 'AddModelFromURDFSrv-response"
  "hebiros/AddModelFromURDFSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddModelFromURDFSrv-response>)))
  "Returns md5sum for a message object of type '<AddModelFromURDFSrv-response>"
  "7503667ca5355c5a12e4de26d45dcf94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddModelFromURDFSrv-response)))
  "Returns md5sum for a message object of type 'AddModelFromURDFSrv-response"
  "7503667ca5355c5a12e4de26d45dcf94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddModelFromURDFSrv-response>)))
  "Returns full string definition for message of type '<AddModelFromURDFSrv-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddModelFromURDFSrv-response)))
  "Returns full string definition for message of type 'AddModelFromURDFSrv-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddModelFromURDFSrv-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddModelFromURDFSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddModelFromURDFSrv-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddModelFromURDFSrv)))
  'AddModelFromURDFSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddModelFromURDFSrv)))
  'AddModelFromURDFSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddModelFromURDFSrv)))
  "Returns string type for a service object of type '<AddModelFromURDFSrv>"
  "hebiros/AddModelFromURDFSrv")