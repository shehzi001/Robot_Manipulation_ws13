
(cl:in-package :asdf)

(defsystem "amtec-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "TargetAcceleration" :depends-on ("_package_TargetAcceleration"))
    (:file "_package_TargetAcceleration" :depends-on ("_package"))
    (:file "Reset" :depends-on ("_package_Reset"))
    (:file "_package_Reset" :depends-on ("_package"))
    (:file "GetStatus" :depends-on ("_package_GetStatus"))
    (:file "_package_GetStatus" :depends-on ("_package"))
    (:file "SweepTilt" :depends-on ("_package_SweepTilt"))
    (:file "_package_SweepTilt" :depends-on ("_package"))
    (:file "Home" :depends-on ("_package_Home"))
    (:file "_package_Home" :depends-on ("_package"))
    (:file "SetVelocity" :depends-on ("_package_SetVelocity"))
    (:file "_package_SetVelocity" :depends-on ("_package"))
    (:file "TargetVelocity" :depends-on ("_package_TargetVelocity"))
    (:file "_package_TargetVelocity" :depends-on ("_package"))
    (:file "Halt" :depends-on ("_package_Halt"))
    (:file "_package_Halt" :depends-on ("_package"))
    (:file "SweepPan" :depends-on ("_package_SweepPan"))
    (:file "_package_SweepPan" :depends-on ("_package"))
    (:file "SetPosition" :depends-on ("_package_SetPosition"))
    (:file "_package_SetPosition" :depends-on ("_package"))
  ))