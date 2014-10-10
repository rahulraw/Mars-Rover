
(cl:in-package :asdf)

(defsystem "cameras-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "video" :depends-on ("_package_video"))
    (:file "_package_video" :depends-on ("_package"))
  ))