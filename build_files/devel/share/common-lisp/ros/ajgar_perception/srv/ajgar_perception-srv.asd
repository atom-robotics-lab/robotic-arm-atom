
(cl:in-package :asdf)

(defsystem "ajgar_perception-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "octomapSrv" :depends-on ("_package_octomapSrv"))
    (:file "_package_octomapSrv" :depends-on ("_package"))
    (:file "percepSrv" :depends-on ("_package_percepSrv"))
    (:file "_package_percepSrv" :depends-on ("_package"))
  ))