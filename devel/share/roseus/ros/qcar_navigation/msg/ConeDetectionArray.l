;; Auto-generated. Do not edit!


(when (boundp 'qcar_navigation::ConeDetectionArray)
  (if (not (find-package "QCAR_NAVIGATION"))
    (make-package "QCAR_NAVIGATION"))
  (shadow 'ConeDetectionArray (find-package "QCAR_NAVIGATION")))
(unless (find-package "QCAR_NAVIGATION::CONEDETECTIONARRAY")
  (make-package "QCAR_NAVIGATION::CONEDETECTIONARRAY"))

(in-package "ROS")
;;//! \htmlinclude ConeDetectionArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass qcar_navigation::ConeDetectionArray
  :super ros::object
  :slots (_header _detections ))

(defmethod qcar_navigation::ConeDetectionArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detections __detections) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _detections __detections)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; qcar_navigation/ConeDetection[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; qcar_navigation/ConeDetection[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; qcar_navigation/ConeDetection[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance qcar_navigation::ConeDetection :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get qcar_navigation::ConeDetectionArray :md5sum-) "ef207be9d16b52faef039f081b24c602")
(setf (get qcar_navigation::ConeDetectionArray :datatype-) "qcar_navigation/ConeDetectionArray")
(setf (get qcar_navigation::ConeDetectionArray :definition-)
      "std_msgs/Header header 

qcar_navigation/ConeDetection[] detections
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: qcar_navigation/ConeDetection
std_msgs/Header header

uint8 UNKNOWN = 0
uint8 BLUE = 1
uint8 YELLOW = 2
uint8 ORANGE = 3
uint8 colour

geometry_msgs/Vector3 direction

float64 range

geometry_msgs/Point position

float64[4] position_covariance

int32 bbox_x
int32 bbox_y
int32 bbox_w
int32 bbox_h

int32 pixel_count
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :qcar_navigation/ConeDetectionArray "ef207be9d16b52faef039f081b24c602")


