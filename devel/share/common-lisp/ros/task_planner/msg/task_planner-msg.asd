
(cl:in-package :asdf)

(defsystem "task_planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Give_Array" :depends-on ("_package_Give_Array"))
    (:file "_package_Give_Array" :depends-on ("_package"))
  ))