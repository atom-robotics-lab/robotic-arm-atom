
(cl:in-package :asdf)

(defsystem "ajgar_core-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "tfValueSrv" :depends-on ("_package_tfValueSrv"))
    (:file "_package_tfValueSrv" :depends-on ("_package"))
  ))