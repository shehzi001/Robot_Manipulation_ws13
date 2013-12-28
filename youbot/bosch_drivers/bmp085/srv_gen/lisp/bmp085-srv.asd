
(cl:in-package :asdf)

(defsystem "bmp085-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "weather_forcast" :depends-on ("_package_weather_forcast"))
    (:file "_package_weather_forcast" :depends-on ("_package"))
    (:file "measure" :depends-on ("_package_measure"))
    (:file "_package_measure" :depends-on ("_package"))
  ))