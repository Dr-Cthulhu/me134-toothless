; Auto-generated. Do not edit!


(cl:in-package hebiros-srv)


;//! \htmlinclude AddGroupFromURDFSrv-request.msg.html

(cl:defclass <AddGroupFromURDFSrv-request> (roslisp-msg-protocol:ros-message)
  ((group_name
    :reader group_name
    :initarg :group_name
    :type cl:string
    :initform ""))
)

(cl:defclass AddGroupFromURDFSrv-request (<AddGroupFromURDFSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddGroupFromURDFSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddGroupFromURDFSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebiros-srv:<AddGroupFromURDFSrv-request> is deprecated: use hebiros-srv:AddGroupFromURDFSrv-request instead.")))

(cl:ensure-generic-function 'group_name-val :lambda-list '(m))
(cl:defmethod group_name-val ((m <AddGroupFromURDFSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebiros-srv:group_name-val is deprecated.  Use hebiros-srv:group_name instead.")
  (group_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddGroupFromURDFSrv-request>) ostream)
  "Serializes a message object of type '<AddGroupFromURDFSrv-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'group_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'group_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddGroupFromURDFSrv-request>) istream)
  "Deserializes a message object of type '<AddGroupFromURDFSrv-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'group_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'group_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddGroupFromURDFSrv-request>)))
  "Returns string type for a service object of type '<AddGroupFromURDFSrv-request>"
  "hebiros/AddGroupFromURDFSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddGroupFromURDFSrv-request)))
  "Returns string type for a service object of type 'AddGroupFromURDFSrv-request"
  "hebiros/AddGroupFromURDFSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddGroupFromURDFSrv-request>)))
  "Returns md5sum for a message object of type '<AddGroupFromURDFSrv-request>"
  "967d0b0c0d858ded8a6a69abbce0c981")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddGroupFromURDFSrv-request)))
  "Returns md5sum for a message object of type 'AddGroupFromURDFSrv-request"
  "967d0b0c0d858ded8a6a69abbce0c981")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddGroupFromURDFSrv-request>)))
  "Returns full string definition for message of type '<AddGroupFromURDFSrv-request>"
  (cl:format cl:nil "string group_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddGroupFromURDFSrv-request)))
  "Returns full string definition for message of type 'AddGroupFromURDFSrv-request"
  (cl:format cl:nil "string group_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddGroupFromURDFSrv-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'group_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddGroupFromURDFSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddGroupFromURDFSrv-request
    (cl:cons ':group_name (group_name msg))
))
;//! \htmlinclude AddGroupFromURDFSrv-response.msg.html

(cl:defclass <AddGroupFromURDFSrv-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass AddGroupFromURDFSrv-response (<AddGroupFromURDFSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddGroupFromURDFSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddGroupFromURDFSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebiros-srv:<AddGroupFromURDFSrv-response> is deprecated: use hebiros-srv:AddGroupFromURDFSrv-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddGroupFromURDFSrv-response>) ostream)
  "Serializes a message object of type '<AddGroupFromURDFSrv-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddGroupFromURDFSrv-response>) istream)
  "Deserializes a message object of type '<AddGroupFromURDFSrv-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddGroupFromURDFSrv-response>)))
  "Returns string type for a service object of type '<AddGroupFromURDFSrv-response>"
  "hebiros/AddGroupFromURDFSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddGroupFromURDFSrv-response)))
  "Returns string type for a service object of type 'AddGroupFromURDFSrv-response"
  "hebiros/AddGroupFromURDFSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddGroupFromURDFSrv-response>)))
  "Returns md5sum for a message object of type '<AddGroupFromURDFSrv-response>"
  "967d0b0c0d858ded8a6a69abbce0c981")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddGroupFromURDFSrv-response)))
  "Returns md5sum for a message object of type 'AddGroupFromURDFSrv-response"
  "967d0b0c0d858ded8a6a69abbce0c981")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddGroupFromURDFSrv-response>)))
  "Returns full string definition for message of type '<AddGroupFromURDFSrv-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddGroupFromURDFSrv-response)))
  "Returns full string definition for message of type 'AddGroupFromURDFSrv-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddGroupFromURDFSrv-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddGroupFromURDFSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddGroupFromURDFSrv-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddGroupFromURDFSrv)))
  'AddGroupFromURDFSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddGroupFromURDFSrv)))
  'AddGroupFromURDFSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddGroupFromURDFSrv)))
  "Returns string type for a service object of type '<AddGroupFromURDFSrv>"
  "hebiros/AddGroupFromURDFSrv")