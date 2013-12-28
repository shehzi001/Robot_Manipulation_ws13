
(cl:in-package :asdf)

(defsystem "bmc050_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "bmc050_measurement" :depends-on ("_package_bmc050_measurement"))
    (:file "_package_bmc050_measurement" :depends-on ("_package"))
  ))