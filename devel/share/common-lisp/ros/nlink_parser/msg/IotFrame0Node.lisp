; Auto-generated. Do not edit!


(cl:in-package nlink_parser-msg)


;//! \htmlinclude IotFrame0Node.msg.html

(cl:defclass <IotFrame0Node> (roslisp-msg-protocol:ros-message)
  ((uid
    :reader uid
    :initarg :uid
    :type cl:integer
    :initform 0)
   (cnt
    :reader cnt
    :initarg :cnt
    :type cl:fixnum
    :initform 0)
   (dis
    :reader dis
    :initarg :dis
    :type cl:float
    :initform 0.0)
   (aoa_angle_horizontal
    :reader aoa_angle_horizontal
    :initarg :aoa_angle_horizontal
    :type cl:float
    :initform 0.0)
   (aoa_angle_vertical
    :reader aoa_angle_vertical
    :initarg :aoa_angle_vertical
    :type cl:float
    :initform 0.0))
)

(cl:defclass IotFrame0Node (<IotFrame0Node>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IotFrame0Node>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IotFrame0Node)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nlink_parser-msg:<IotFrame0Node> is deprecated: use nlink_parser-msg:IotFrame0Node instead.")))

(cl:ensure-generic-function 'uid-val :lambda-list '(m))
(cl:defmethod uid-val ((m <IotFrame0Node>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nlink_parser-msg:uid-val is deprecated.  Use nlink_parser-msg:uid instead.")
  (uid m))

(cl:ensure-generic-function 'cnt-val :lambda-list '(m))
(cl:defmethod cnt-val ((m <IotFrame0Node>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nlink_parser-msg:cnt-val is deprecated.  Use nlink_parser-msg:cnt instead.")
  (cnt m))

(cl:ensure-generic-function 'dis-val :lambda-list '(m))
(cl:defmethod dis-val ((m <IotFrame0Node>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nlink_parser-msg:dis-val is deprecated.  Use nlink_parser-msg:dis instead.")
  (dis m))

(cl:ensure-generic-function 'aoa_angle_horizontal-val :lambda-list '(m))
(cl:defmethod aoa_angle_horizontal-val ((m <IotFrame0Node>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nlink_parser-msg:aoa_angle_horizontal-val is deprecated.  Use nlink_parser-msg:aoa_angle_horizontal instead.")
  (aoa_angle_horizontal m))

(cl:ensure-generic-function 'aoa_angle_vertical-val :lambda-list '(m))
(cl:defmethod aoa_angle_vertical-val ((m <IotFrame0Node>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nlink_parser-msg:aoa_angle_vertical-val is deprecated.  Use nlink_parser-msg:aoa_angle_vertical instead.")
  (aoa_angle_vertical m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IotFrame0Node>) ostream)
  "Serializes a message object of type '<IotFrame0Node>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cnt)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dis))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'aoa_angle_horizontal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'aoa_angle_vertical))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IotFrame0Node>) istream)
  "Deserializes a message object of type '<IotFrame0Node>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cnt)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dis) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'aoa_angle_horizontal) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'aoa_angle_vertical) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IotFrame0Node>)))
  "Returns string type for a message object of type '<IotFrame0Node>"
  "nlink_parser/IotFrame0Node")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IotFrame0Node)))
  "Returns string type for a message object of type 'IotFrame0Node"
  "nlink_parser/IotFrame0Node")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IotFrame0Node>)))
  "Returns md5sum for a message object of type '<IotFrame0Node>"
  "18c74fd1a18895f584bf54c3563d0a60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IotFrame0Node)))
  "Returns md5sum for a message object of type 'IotFrame0Node"
  "18c74fd1a18895f584bf54c3563d0a60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IotFrame0Node>)))
  "Returns full string definition for message of type '<IotFrame0Node>"
  (cl:format cl:nil "uint32 uid~%uint8 cnt~%float32 dis~%float32 aoa_angle_horizontal~%float32 aoa_angle_vertical~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IotFrame0Node)))
  "Returns full string definition for message of type 'IotFrame0Node"
  (cl:format cl:nil "uint32 uid~%uint8 cnt~%float32 dis~%float32 aoa_angle_horizontal~%float32 aoa_angle_vertical~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IotFrame0Node>))
  (cl:+ 0
     4
     1
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IotFrame0Node>))
  "Converts a ROS message object to a list"
  (cl:list 'IotFrame0Node
    (cl:cons ':uid (uid msg))
    (cl:cons ':cnt (cnt msg))
    (cl:cons ':dis (dis msg))
    (cl:cons ':aoa_angle_horizontal (aoa_angle_horizontal msg))
    (cl:cons ':aoa_angle_vertical (aoa_angle_vertical msg))
))
