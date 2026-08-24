
(cl:in-package :asdf)

(defsystem "qcar_guidance-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "TrajectoryMessage" :depends-on ("_package_TrajectoryMessage"))
    (:file "_package_TrajectoryMessage" :depends-on ("_package"))
  ))