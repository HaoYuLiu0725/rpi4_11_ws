;; Auto-generated. Do not edit!


(when (boundp 'nlink_parser::IotFrame0)
  (if (not (find-package "NLINK_PARSER"))
    (make-package "NLINK_PARSER"))
  (shadow 'IotFrame0 (find-package "NLINK_PARSER")))
(unless (find-package "NLINK_PARSER::IOTFRAME0")
  (make-package "NLINK_PARSER::IOTFRAME0"))

(in-package "ROS")
;;//! \htmlinclude IotFrame0.msg.html


(defclass nlink_parser::IotFrame0
  :super ros::object
  :slots (_uid _nodes ))

(defmethod nlink_parser::IotFrame0
  (:init
   (&key
    ((:uid __uid) 0)
    ((:nodes __nodes) ())
    )
   (send-super :init)
   (setq _uid (round __uid))
   (setq _nodes __nodes)
   self)
  (:uid
   (&optional __uid)
   (if __uid (setq _uid __uid)) _uid)
  (:nodes
   (&rest __nodes)
   (if (keywordp (car __nodes))
       (send* _nodes __nodes)
     (progn
       (if __nodes (setq _nodes (car __nodes)))
       _nodes)))
  (:serialization-length
   ()
   (+
    ;; uint32 _uid
    4
    ;; nlink_parser/IotFrame0Node[] _nodes
    (apply #'+ (send-all _nodes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _uid
       (write-long _uid s)
     ;; nlink_parser/IotFrame0Node[] _nodes
     (write-long (length _nodes) s)
     (dolist (elem _nodes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _uid
     (setq _uid (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; nlink_parser/IotFrame0Node[] _nodes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _nodes (let (r) (dotimes (i n) (push (instance nlink_parser::IotFrame0Node :init) r)) r))
     (dolist (elem- _nodes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get nlink_parser::IotFrame0 :md5sum-) "5942449595b10adf20faeffdc7ac5270")
(setf (get nlink_parser::IotFrame0 :datatype-) "nlink_parser/IotFrame0")
(setf (get nlink_parser::IotFrame0 :definition-)
      "uint32 uid
IotFrame0Node[] nodes

================================================================================
MSG: nlink_parser/IotFrame0Node
uint32 uid
uint8 cnt
float32 dis
float32 aoa_angle_horizontal
float32 aoa_angle_vertical

")



(provide :nlink_parser/IotFrame0 "5942449595b10adf20faeffdc7ac5270")


