
(cl:in-package :asdf)

(defsystem "cameras-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GrayImage" :depends-on ("_package_GrayImage"))
    (:file "_package_GrayImage" :depends-on ("_package"))
  ))