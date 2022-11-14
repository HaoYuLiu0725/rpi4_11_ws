;; Auto-generated. Do not edit!


(when (boundp 'arm_move::mission)
  (if (not (find-package "ARM_MOVE"))
    (make-package "ARM_MOVE"))
  (shadow 'mission (find-package "ARM_MOVE")))
(unless (find-package "ARM_MOVE::MISSION")
  (make-package "ARM_MOVE::MISSION"))

(in-package "ROS")
;;//! \htmlinclude mission.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass arm_move::mission
  :super ros::object
  :slots (_type _T _E _L ))

(defmethod arm_move::mission
  (:init
   (&key
    ((:type __type) 0)
    ((:T __T) (instance geometry_msgs::Point :init))
    ((:E __E) (instance geometry_msgs::Point :init))
    ((:L __L) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _T __T)
   (setq _E __E)
   (setq _L __L)
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:T
   (&rest __T)
   (if (keywordp (car __T))
       (send* _T __T)
     (progn
       (if __T (setq _T (car __T)))
       _T)))
  (:E
   (&rest __E)
   (if (keywordp (car __E))
       (send* _E __E)
     (progn
       (if __E (setq _E (car __E)))
       _E)))
  (:L
   (&rest __L)
   (if (keywordp (car __L))
       (send* _L __L)
     (progn
       (if __L (setq _L (car __L)))
       _L)))
  (:serialization-length
   ()
   (+
    ;; int32 _type
    4
    ;; geometry_msgs/Point _T
    (send _T :serialization-length)
    ;; geometry_msgs/Point _E
    (send _E :serialization-length)
    ;; geometry_msgs/Point _L
    (send _L :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _type
       (write-long _type s)
     ;; geometry_msgs/Point _T
       (send _T :serialize s)
     ;; geometry_msgs/Point _E
       (send _E :serialize s)
     ;; geometry_msgs/Point _L
       (send _L :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point _T
     (send _T :deserialize buf ptr-) (incf ptr- (send _T :serialization-length))
   ;; geometry_msgs/Point _E
     (send _E :deserialize buf ptr-) (incf ptr- (send _E :serialization-length))
   ;; geometry_msgs/Point _L
     (send _L :deserialize buf ptr-) (incf ptr- (send _L :serialization-length))
   ;;
   self)
  )

(setf (get arm_move::mission :md5sum-) "1ebf46cc34619c6ab6b248306e7f7e41")
(setf (get arm_move::mission :datatype-) "arm_move/mission")
(setf (get arm_move::mission :definition-)
      "int32 type
geometry_msgs/Point T
geometry_msgs/Point E
geometry_msgs/Point L
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :arm_move/mission "1ebf46cc34619c6ab6b248306e7f7e41")


