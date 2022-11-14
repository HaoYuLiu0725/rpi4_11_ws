
(cl:in-package :asdf)

(defsystem "arm_move-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "mission" :depends-on ("_package_mission"))
    (:file "_package_mission" :depends-on ("_package"))
  ))