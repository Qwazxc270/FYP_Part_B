
(cl:in-package :asdf)

(defsystem "qcar_navigation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ConeDetection" :depends-on ("_package_ConeDetection"))
    (:file "_package_ConeDetection" :depends-on ("_package"))
    (:file "ConeDetectionArray" :depends-on ("_package_ConeDetectionArray"))
    (:file "_package_ConeDetectionArray" :depends-on ("_package"))
  ))