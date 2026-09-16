; Auto-generated. Do not edit!


(cl:in-package qcar_navigation-msg)


;//! \htmlinclude ConeDetection.msg.html

(cl:defclass <ConeDetection> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (colour
    :reader colour
    :initarg :colour
    :type cl:fixnum
    :initform 0)
   (direction
    :reader direction
    :initarg :direction
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (range
    :reader range
    :initarg :range
    :type cl:float
    :initform 0.0)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (position_covariance
    :reader position_covariance
    :initarg :position_covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (bbox_x
    :reader bbox_x
    :initarg :bbox_x
    :type cl:integer
    :initform 0)
   (bbox_y
    :reader bbox_y
    :initarg :bbox_y
    :type cl:integer
    :initform 0)
   (bbox_w
    :reader bbox_w
    :initarg :bbox_w
    :type cl:integer
    :initform 0)
   (bbox_h
    :reader bbox_h
    :initarg :bbox_h
    :type cl:integer
    :initform 0)
   (pixel_count
    :reader pixel_count
    :initarg :pixel_count
    :type cl:integer
    :initform 0))
)

(cl:defclass ConeDetection (<ConeDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConeDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConeDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qcar_navigation-msg:<ConeDetection> is deprecated: use qcar_navigation-msg:ConeDetection instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ConeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qcar_navigation-msg:header-val is deprecated.  Use qcar_navigation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'colour-val :lambda-list '(m))
(cl:defmethod colour-val ((m <ConeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qcar_navigation-msg:colour-val is deprecated.  Use qcar_navigation-msg:colour instead.")
  (colour m))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <ConeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qcar_navigation-msg:direction-val is deprecated.  Use qcar_navigation-msg:direction instead.")
  (direction m))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <ConeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qcar_navigation-msg:range-val is deprecated.  Use qcar_navigation-msg:range instead.")
  (range m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <ConeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qcar_navigation-msg:position-val is deprecated.  Use qcar_navigation-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'position_covariance-val :lambda-list '(m))
(cl:defmethod position_covariance-val ((m <ConeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qcar_navigation-msg:position_covariance-val is deprecated.  Use qcar_navigation-msg:position_covariance instead.")
  (position_covariance m))

(cl:ensure-generic-function 'bbox_x-val :lambda-list '(m))
(cl:defmethod bbox_x-val ((m <ConeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qcar_navigation-msg:bbox_x-val is deprecated.  Use qcar_navigation-msg:bbox_x instead.")
  (bbox_x m))

(cl:ensure-generic-function 'bbox_y-val :lambda-list '(m))
(cl:defmethod bbox_y-val ((m <ConeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qcar_navigation-msg:bbox_y-val is deprecated.  Use qcar_navigation-msg:bbox_y instead.")
  (bbox_y m))

(cl:ensure-generic-function 'bbox_w-val :lambda-list '(m))
(cl:defmethod bbox_w-val ((m <ConeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qcar_navigation-msg:bbox_w-val is deprecated.  Use qcar_navigation-msg:bbox_w instead.")
  (bbox_w m))

(cl:ensure-generic-function 'bbox_h-val :lambda-list '(m))
(cl:defmethod bbox_h-val ((m <ConeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qcar_navigation-msg:bbox_h-val is deprecated.  Use qcar_navigation-msg:bbox_h instead.")
  (bbox_h m))

(cl:ensure-generic-function 'pixel_count-val :lambda-list '(m))
(cl:defmethod pixel_count-val ((m <ConeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qcar_navigation-msg:pixel_count-val is deprecated.  Use qcar_navigation-msg:pixel_count instead.")
  (pixel_count m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ConeDetection>)))
    "Constants for message type '<ConeDetection>"
  '((:UNKNOWN . 0)
    (:BLUE . 1)
    (:YELLOW . 2)
    (:ORANGE . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ConeDetection)))
    "Constants for message type 'ConeDetection"
  '((:UNKNOWN . 0)
    (:BLUE . 1)
    (:YELLOW . 2)
    (:ORANGE . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConeDetection>) ostream)
  "Serializes a message object of type '<ConeDetection>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'colour)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'direction) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'position_covariance))
  (cl:let* ((signed (cl:slot-value msg 'bbox_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'bbox_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'bbox_w)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'bbox_h)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'pixel_count)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConeDetection>) istream)
  "Deserializes a message object of type '<ConeDetection>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'colour)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'direction) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (cl:setf (cl:slot-value msg 'position_covariance) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'position_covariance)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bbox_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bbox_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bbox_w) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bbox_h) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pixel_count) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConeDetection>)))
  "Returns string type for a message object of type '<ConeDetection>"
  "qcar_navigation/ConeDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConeDetection)))
  "Returns string type for a message object of type 'ConeDetection"
  "qcar_navigation/ConeDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConeDetection>)))
  "Returns md5sum for a message object of type '<ConeDetection>"
  "41df40de730901088a49c65ee720b31e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConeDetection)))
  "Returns md5sum for a message object of type 'ConeDetection"
  "41df40de730901088a49c65ee720b31e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConeDetection>)))
  "Returns full string definition for message of type '<ConeDetection>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 UNKNOWN = 0~%uint8 BLUE = 1~%uint8 YELLOW = 2~%uint8 ORANGE = 3~%uint8 colour~%~%geometry_msgs/Vector3 direction~%~%float64 range~%~%geometry_msgs/Point position~%~%float64[4] position_covariance~%~%int32 bbox_x~%int32 bbox_y~%int32 bbox_w~%int32 bbox_h~%~%int32 pixel_count~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConeDetection)))
  "Returns full string definition for message of type 'ConeDetection"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 UNKNOWN = 0~%uint8 BLUE = 1~%uint8 YELLOW = 2~%uint8 ORANGE = 3~%uint8 colour~%~%geometry_msgs/Vector3 direction~%~%float64 range~%~%geometry_msgs/Point position~%~%float64[4] position_covariance~%~%int32 bbox_x~%int32 bbox_y~%int32 bbox_w~%int32 bbox_h~%~%int32 pixel_count~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConeDetection>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'direction))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'position_covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConeDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'ConeDetection
    (cl:cons ':header (header msg))
    (cl:cons ':colour (colour msg))
    (cl:cons ':direction (direction msg))
    (cl:cons ':range (range msg))
    (cl:cons ':position (position msg))
    (cl:cons ':position_covariance (position_covariance msg))
    (cl:cons ':bbox_x (bbox_x msg))
    (cl:cons ':bbox_y (bbox_y msg))
    (cl:cons ':bbox_w (bbox_w msg))
    (cl:cons ':bbox_h (bbox_h msg))
    (cl:cons ':pixel_count (pixel_count msg))
))
