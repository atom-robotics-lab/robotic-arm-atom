
(cl:in-package :asdf)

(defsystem "plugin_pneumatic_gripper-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Attach" :depends-on ("_package_Attach"))
    (:file "_package_Attach" :depends-on ("_package"))
  ))