
(cl:in-package :asdf)

(defsystem "bmp085-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "bmp085_measurement" :depends-on ("_package_bmp085_measurement"))
    (:file "_package_bmp085_measurement" :depends-on ("_package"))
  ))