
(cl:in-package :asdf)

(defsystem "joystick_packages-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "JoystickMsg" :depends-on ("_package_JoystickMsg"))
    (:file "_package_JoystickMsg" :depends-on ("_package"))
  ))