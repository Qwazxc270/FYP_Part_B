; Auto-generated. Do not edit!


(cl:in-package qcar_navigation-msg)


;//! \htmlinclude ConeDetectionArray.msg.html

(cl:defclass <ConeDetectionArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detections
    :reader detections
    :initarg :detections
    :type (cl:vector qcar_navigation-msg:ConeDetection)
   :initform (cl:make-array 0 :element-type 'qcar_navigation-msg:ConeDetection :initial-element (cl:make-instance 'qcar_navigation-msg:ConeDetection))))
)

(cl:defclass ConeDetectionArray (<ConeDetectionArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConeDetectionArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConeDetectionArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qcar_navigation-msg:<ConeDetectionArray> is deprecated: use qcar_navigation-msg:ConeDetectionArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ConeDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qcar_navigation-msg:header-val is deprecated.  Use qcar_navigation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <ConeDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qcar_navigation-msg:detections-val is deprecated.  Use qcar_navigation-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConeDetectionArray>) ostream)
  "Serializes a message object of type '<ConeDetectionArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConeDetectionArray>) istream)
  "Deserializes a message object of type '<ConeDetectionArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'qcar_navigation-msg:ConeDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConeDetectionArray>)))
  "Returns string type for a message object of type '<ConeDetectionArray>"
  "qcar_navigation/ConeDetectionArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConeDetectionArray)))
  "Returns string type for a message object of type 'ConeDetectionArray"
  "qcar_navigation/ConeDetectionArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConeDetectionArray>)))
  "Returns md5sum for a message object of type '<ConeDetectionArray>"
  "ef207be9d16b52faef039f081b24c602")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConeDetectionArray)))
  "Returns md5sum for a message object of type 'ConeDetectionArray"
  "ef207be9d16b52faef039f081b24c602")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConeDetectionArray>)))
  "Returns full string definition for message of type '<ConeDetectionArray>"
  (cl:format cl:nil "std_msgs/Header header ~%~%qcar_navigation/ConeDetection[] detections~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: qcar_navigation/ConeDetection~%std_msgs/Header header~%~%uint8 UNKNOWN = 0~%uint8 BLUE = 1~%uint8 YELLOW = 2~%uint8 ORANGE = 3~%uint8 colour~%~%geometry_msgs/Vector3 direction~%~%float64 range~%~%geometry_msgs/Point position~%~%float64[4] position_covariance~%~%int32 bbox_x~%int32 bbox_y~%int32 bbox_w~%int32 bbox_h~%~%int32 pixel_count~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConeDetectionArray)))
  "Returns full string definition for message of type 'ConeDetectionArray"
  (cl:format cl:nil "std_msgs/Header header ~%~%qcar_navigation/ConeDetection[] detections~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: qcar_navigation/ConeDetection~%std_msgs/Header header~%~%uint8 UNKNOWN = 0~%uint8 BLUE = 1~%uint8 YELLOW = 2~%uint8 ORANGE = 3~%uint8 colour~%~%geometry_msgs/Vector3 direction~%~%float64 range~%~%geometry_msgs/Point position~%~%float64[4] position_covariance~%~%int32 bbox_x~%int32 bbox_y~%int32 bbox_w~%int32 bbox_h~%~%int32 pixel_count~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConeDetectionArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConeDetectionArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ConeDetectionArray
    (cl:cons ':header (header msg))
    (cl:cons ':detections (detections msg))
))
