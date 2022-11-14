; Auto-generated. Do not edit!


(cl:in-package arm_move-msg)


;//! \htmlinclude mission.msg.html

(cl:defclass <mission> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (T
    :reader T
    :initarg :T
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (E
    :reader E
    :initarg :E
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (L
    :reader L
    :initarg :L
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass mission (<mission>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mission>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mission)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm_move-msg:<mission> is deprecated: use arm_move-msg:mission instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <mission>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_move-msg:type-val is deprecated.  Use arm_move-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'T-val :lambda-list '(m))
(cl:defmethod T-val ((m <mission>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_move-msg:T-val is deprecated.  Use arm_move-msg:T instead.")
  (T m))

(cl:ensure-generic-function 'E-val :lambda-list '(m))
(cl:defmethod E-val ((m <mission>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_move-msg:E-val is deprecated.  Use arm_move-msg:E instead.")
  (E m))

(cl:ensure-generic-function 'L-val :lambda-list '(m))
(cl:defmethod L-val ((m <mission>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_move-msg:L-val is deprecated.  Use arm_move-msg:L instead.")
  (L m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mission>) ostream)
  "Serializes a message object of type '<mission>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'T) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'E) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'L) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mission>) istream)
  "Deserializes a message object of type '<mission>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'T) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'E) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'L) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mission>)))
  "Returns string type for a message object of type '<mission>"
  "arm_move/mission")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mission)))
  "Returns string type for a message object of type 'mission"
  "arm_move/mission")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mission>)))
  "Returns md5sum for a message object of type '<mission>"
  "1ebf46cc34619c6ab6b248306e7f7e41")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mission)))
  "Returns md5sum for a message object of type 'mission"
  "1ebf46cc34619c6ab6b248306e7f7e41")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mission>)))
  "Returns full string definition for message of type '<mission>"
  (cl:format cl:nil "int32 type~%geometry_msgs/Point T~%geometry_msgs/Point E~%geometry_msgs/Point L~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mission)))
  "Returns full string definition for message of type 'mission"
  (cl:format cl:nil "int32 type~%geometry_msgs/Point T~%geometry_msgs/Point E~%geometry_msgs/Point L~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mission>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'T))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'E))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'L))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mission>))
  "Converts a ROS message object to a list"
  (cl:list 'mission
    (cl:cons ':type (type msg))
    (cl:cons ':T (T msg))
    (cl:cons ':E (E msg))
    (cl:cons ':L (L msg))
))
