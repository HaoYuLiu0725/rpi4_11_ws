;; Auto-generated. Do not edit!


(when (boundp 'nlink_parser::IotFrame0Node)
  (if (not (find-package "NLINK_PARSER"))
    (make-package "NLINK_PARSER"))
  (shadow 'IotFrame0Node (find-package "NLINK_PARSER")))
(unless (find-package "NLINK_PARSER::IOTFRAME0NODE")
  (make-package "NLINK_PARSER::IOTFRAME0NODE"))

(in-package "ROS")
;;//! \htmlinclude IotFrame0Node.msg.html


(defclass nlink_parser::IotFrame0Node
  :super ros::object
  :slots (_uid _cnt _dis _aoa_angle_horizontal _aoa_angle_vertical ))

(defmethod nlink_parser::IotFrame0Node
  (:init
   (&key
    ((:uid __uid) 0)
    ((:cnt __cnt) 0)
    ((:dis __dis) 0.0)
    ((:aoa_angle_horizontal __aoa_angle_horizontal) 0.0)
    ((:aoa_angle_vertical __aoa_angle_vertical) 0.0)
    )
   (send-super :init)
   (setq _uid (round __uid))
   (setq _cnt (round __cnt))
   (setq _dis (float __dis))
   (setq _aoa_angle_horizontal (float __aoa_angle_horizontal))
   (setq _aoa_angle_vertical (float __aoa_angle_vertical))
   self)
  (:uid
   (&optional __uid)
   (if __uid (setq _uid __uid)) _uid)
  (:cnt
   (&optional __cnt)
   (if __cnt (setq _cnt __cnt)) _cnt)
  (:dis
   (&optional __dis)
   (if __dis (setq _dis __dis)) _dis)
  (:aoa_angle_horizontal
   (&optional __aoa_angle_horizontal)
   (if __aoa_angle_horizontal (setq _aoa_angle_horizontal __aoa_angle_horizontal)) _aoa_angle_horizontal)
  (:aoa_angle_vertical
   (&optional __aoa_angle_vertical)
   (if __aoa_angle_vertical (setq _aoa_angle_vertical __aoa_angle_vertical)) _aoa_angle_vertical)
  (:serialization-length
   ()
   (+
    ;; uint32 _uid
    4
    ;; uint8 _cnt
    1
    ;; float32 _dis
    4
    ;; float32 _aoa_angle_horizontal
    4
    ;; float32 _aoa_angle_vertical
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _uid
       (write-long _uid s)
     ;; uint8 _cnt
       (write-byte _cnt s)
     ;; float32 _dis
       (sys::poke _dis (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _aoa_angle_horizontal
       (sys::poke _aoa_angle_horizontal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _aoa_angle_vertical
       (sys::poke _aoa_angle_vertical (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _uid
     (setq _uid (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _cnt
     (setq _cnt (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _dis
     (setq _dis (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _aoa_angle_horizontal
     (setq _aoa_angle_horizontal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _aoa_angle_vertical
     (setq _aoa_angle_vertical (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get nlink_parser::IotFrame0Node :md5sum-) "18c74fd1a18895f584bf54c3563d0a60")
(setf (get nlink_parser::IotFrame0Node :datatype-) "nlink_parser/IotFrame0Node")
(setf (get nlink_parser::IotFrame0Node :definition-)
      "uint32 uid
uint8 cnt
float32 dis
float32 aoa_angle_horizontal
float32 aoa_angle_vertical

")



(provide :nlink_parser/IotFrame0Node "18c74fd1a18895f584bf54c3563d0a60")


