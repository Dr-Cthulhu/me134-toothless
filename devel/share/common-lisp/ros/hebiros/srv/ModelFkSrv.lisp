; Auto-generated. Do not edit!


(cl:in-package hebiros-srv)


;//! \htmlinclude ModelFkSrv-request.msg.html

(cl:defclass <ModelFkSrv-request> (roslisp-msg-protocol:ros-message)
  ((positions
    :reader positions
    :initarg :positions
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (frame_type
    :reader frame_type
    :initarg :frame_type
    :type cl:integer
    :initform 0))
)

(cl:defclass ModelFkSrv-request (<ModelFkSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModelFkSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModelFkSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebiros-srv:<ModelFkSrv-request> is deprecated: use hebiros-srv:ModelFkSrv-request instead.")))

(cl:ensure-generic-function 'positions-val :lambda-list '(m))
(cl:defmethod positions-val ((m <ModelFkSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebiros-srv:positions-val is deprecated.  Use hebiros-srv:positions instead.")
  (positions m))

(cl:ensure-generic-function 'frame_type-val :lambda-list '(m))
(cl:defmethod frame_type-val ((m <ModelFkSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebiros-srv:frame_type-val is deprecated.  Use hebiros-srv:frame_type instead.")
  (frame_type m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ModelFkSrv-request>)))
    "Constants for message type '<ModelFkSrv-request>"
  '((:FRAMETYPECENTEROFMASS . 0)
    (:FRAMETYPEOUTPUT . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ModelFkSrv-request)))
    "Constants for message type 'ModelFkSrv-request"
  '((:FRAMETYPECENTEROFMASS . 0)
    (:FRAMETYPEOUTPUT . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModelFkSrv-request>) ostream)
  "Serializes a message object of type '<ModelFkSrv-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'positions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'positions))
  (cl:let* ((signed (cl:slot-value msg 'frame_type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModelFkSrv-request>) istream)
  "Deserializes a message object of type '<ModelFkSrv-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'positions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'positions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frame_type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModelFkSrv-request>)))
  "Returns string type for a service object of type '<ModelFkSrv-request>"
  "hebiros/ModelFkSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModelFkSrv-request)))
  "Returns string type for a service object of type 'ModelFkSrv-request"
  "hebiros/ModelFkSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModelFkSrv-request>)))
  "Returns md5sum for a message object of type '<ModelFkSrv-request>"
  "0a7e86c595699ff4b039262ca7d90490")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModelFkSrv-request)))
  "Returns md5sum for a message object of type 'ModelFkSrv-request"
  "0a7e86c595699ff4b039262ca7d90490")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModelFkSrv-request>)))
  "Returns full string definition for message of type '<ModelFkSrv-request>"
  (cl:format cl:nil "int32 FrameTypeCenterOfMass = 0~%int32 FrameTypeOutput = 1~%float64[] positions~%int32 frame_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModelFkSrv-request)))
  "Returns full string definition for message of type 'ModelFkSrv-request"
  (cl:format cl:nil "int32 FrameTypeCenterOfMass = 0~%int32 FrameTypeOutput = 1~%float64[] positions~%int32 frame_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModelFkSrv-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'positions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModelFkSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ModelFkSrv-request
    (cl:cons ':positions (positions msg))
    (cl:cons ':frame_type (frame_type msg))
))
;//! \htmlinclude ModelFkSrv-response.msg.html

(cl:defclass <ModelFkSrv-response> (roslisp-msg-protocol:ros-message)
  ((frames
    :reader frames
    :initarg :frames
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ModelFkSrv-response (<ModelFkSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModelFkSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModelFkSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hebiros-srv:<ModelFkSrv-response> is deprecated: use hebiros-srv:ModelFkSrv-response instead.")))

(cl:ensure-generic-function 'frames-val :lambda-list '(m))
(cl:defmethod frames-val ((m <ModelFkSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hebiros-srv:frames-val is deprecated.  Use hebiros-srv:frames instead.")
  (frames m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModelFkSrv-response>) ostream)
  "Serializes a message object of type '<ModelFkSrv-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'frames))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'frames))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModelFkSrv-response>) istream)
  "Deserializes a message object of type '<ModelFkSrv-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'frames) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'frames)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModelFkSrv-response>)))
  "Returns string type for a service object of type '<ModelFkSrv-response>"
  "hebiros/ModelFkSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModelFkSrv-response)))
  "Returns string type for a service object of type 'ModelFkSrv-response"
  "hebiros/ModelFkSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModelFkSrv-response>)))
  "Returns md5sum for a message object of type '<ModelFkSrv-response>"
  "0a7e86c595699ff4b039262ca7d90490")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModelFkSrv-response)))
  "Returns md5sum for a message object of type 'ModelFkSrv-response"
  "0a7e86c595699ff4b039262ca7d90490")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModelFkSrv-response>)))
  "Returns full string definition for message of type '<ModelFkSrv-response>"
  (cl:format cl:nil "float64[] frames~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModelFkSrv-response)))
  "Returns full string definition for message of type 'ModelFkSrv-response"
  (cl:format cl:nil "float64[] frames~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModelFkSrv-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'frames) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModelFkSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ModelFkSrv-response
    (cl:cons ':frames (frames msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ModelFkSrv)))
  'ModelFkSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ModelFkSrv)))
  'ModelFkSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModelFkSrv)))
  "Returns string type for a service object of type '<ModelFkSrv>"
  "hebiros/ModelFkSrv")