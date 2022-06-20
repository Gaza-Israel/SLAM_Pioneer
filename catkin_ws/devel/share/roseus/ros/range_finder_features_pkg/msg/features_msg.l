;; Auto-generated. Do not edit!


(when (boundp 'range_finder_features_pkg::features_msg)
  (if (not (find-package "RANGE_FINDER_FEATURES_PKG"))
    (make-package "RANGE_FINDER_FEATURES_PKG"))
  (shadow 'features_msg (find-package "RANGE_FINDER_FEATURES_PKG")))
(unless (find-package "RANGE_FINDER_FEATURES_PKG::FEATURES_MSG")
  (make-package "RANGE_FINDER_FEATURES_PKG::FEATURES_MSG"))

(in-package "ROS")
;;//! \htmlinclude features_msg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass range_finder_features_pkg::features_msg
  :super ros::object
  :slots (_points _features_updated ))

(defmethod range_finder_features_pkg::features_msg
  (:init
   (&key
    ((:points __points) ())
    ((:features_updated __features_updated) nil)
    )
   (send-super :init)
   (setq _points __points)
   (setq _features_updated __features_updated)
   self)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:features_updated
   (&optional (__features_updated :null))
   (if (not (eq __features_updated :null)) (setq _features_updated __features_updated)) _features_updated)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ;; bool _features_updated
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;; bool _features_updated
       (if _features_updated (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _features_updated
     (setq _features_updated (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get range_finder_features_pkg::features_msg :md5sum-) "3fb405e9ac52e34e46d6e8b78d1a3ac5")
(setf (get range_finder_features_pkg::features_msg :datatype-) "range_finder_features_pkg/features_msg")
(setf (get range_finder_features_pkg::features_msg :definition-)
      "geometry_msgs/Point[] points
bool features_updated
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :range_finder_features_pkg/features_msg "3fb405e9ac52e34e46d6e8b78d1a3ac5")


