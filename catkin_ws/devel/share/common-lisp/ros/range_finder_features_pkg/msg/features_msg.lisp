; Auto-generated. Do not edit!


(cl:in-package range_finder_features_pkg-msg)


;//! \htmlinclude features_msg.msg.html

(cl:defclass <features_msg> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (features_updated
    :reader features_updated
    :initarg :features_updated
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass features_msg (<features_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <features_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'features_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name range_finder_features_pkg-msg:<features_msg> is deprecated: use range_finder_features_pkg-msg:features_msg instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <features_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader range_finder_features_pkg-msg:points-val is deprecated.  Use range_finder_features_pkg-msg:points instead.")
  (points m))

(cl:ensure-generic-function 'features_updated-val :lambda-list '(m))
(cl:defmethod features_updated-val ((m <features_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader range_finder_features_pkg-msg:features_updated-val is deprecated.  Use range_finder_features_pkg-msg:features_updated instead.")
  (features_updated m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <features_msg>) ostream)
  "Serializes a message object of type '<features_msg>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'features_updated) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <features_msg>) istream)
  "Deserializes a message object of type '<features_msg>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:slot-value msg 'features_updated) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<features_msg>)))
  "Returns string type for a message object of type '<features_msg>"
  "range_finder_features_pkg/features_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'features_msg)))
  "Returns string type for a message object of type 'features_msg"
  "range_finder_features_pkg/features_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<features_msg>)))
  "Returns md5sum for a message object of type '<features_msg>"
  "3fb405e9ac52e34e46d6e8b78d1a3ac5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'features_msg)))
  "Returns md5sum for a message object of type 'features_msg"
  "3fb405e9ac52e34e46d6e8b78d1a3ac5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<features_msg>)))
  "Returns full string definition for message of type '<features_msg>"
  (cl:format cl:nil "geometry_msgs/Point[] points~%bool features_updated~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'features_msg)))
  "Returns full string definition for message of type 'features_msg"
  (cl:format cl:nil "geometry_msgs/Point[] points~%bool features_updated~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <features_msg>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <features_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'features_msg
    (cl:cons ':points (points msg))
    (cl:cons ':features_updated (features_updated msg))
))
