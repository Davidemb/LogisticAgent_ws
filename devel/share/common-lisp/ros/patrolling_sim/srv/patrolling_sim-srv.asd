
(cl:in-package :asdf)

(defsystem "patrolling_sim-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GoToStartPosSrv" :depends-on ("_package_GoToStartPosSrv"))
    (:file "_package_GoToStartPosSrv" :depends-on ("_package"))
  ))