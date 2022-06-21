
(cl:in-package :asdf)

(defsystem "range_finder_features_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "features_msg" :depends-on ("_package_features_msg"))
    (:file "_package_features_msg" :depends-on ("_package"))
  ))