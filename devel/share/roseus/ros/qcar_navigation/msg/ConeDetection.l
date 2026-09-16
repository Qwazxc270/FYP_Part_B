;; Auto-generated. Do not edit!


(when (boundp 'qcar_navigation::ConeDetection)
  (if (not (find-package "QCAR_NAVIGATION"))
    (make-package "QCAR_NAVIGATION"))
  (shadow 'ConeDetection (find-package "QCAR_NAVIGATION")))
(unless (find-package "QCAR_NAVIGATION::CONEDETECTION")
  (make-package "QCAR_NAVIGATION::CONEDETECTION"))

(in-package "ROS")
;;//! \htmlinclude ConeDetection.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*UNKNOWN*" (find-package "QCAR_NAVIGATION::CONEDETECTION"))
(shadow '*UNKNOWN* (find-package "QCAR_NAVIGATION::CONEDETECTION"))
(defconstant qcar_navigation::ConeDetection::*UNKNOWN* 0)
(intern "*BLUE*" (find-package "QCAR_NAVIGATION::CONEDETECTION"))
(shadow '*BLUE* (find-package "QCAR_NAVIGATION::CONEDETECTION"))
(defconstant qcar_navigation::ConeDetection::*BLUE* 1)
(intern "*YELLOW*" (find-package "QCAR_NAVIGATION::CONEDETECTION"))
(shadow '*YELLOW* (find-package "QCAR_NAVIGATION::CONEDETECTION"))
(defconstant qcar_navigation::ConeDetection::*YELLOW* 2)
(intern "*ORANGE*" (find-package "QCAR_NAVIGATION::CONEDETECTION"))
(shadow '*ORANGE* (find-package "QCAR_NAVIGATION::CONEDETECTION"))
(defconstant qcar_navigation::ConeDetection::*ORANGE* 3)

(defun qcar_navigation::ConeDetection-to-symbol (const)
  (cond
        ((= const 0) 'qcar_navigation::ConeDetection::*UNKNOWN*)
        ((= const 1) 'qcar_navigation::ConeDetection::*BLUE*)
        ((= const 2) 'qcar_navigation::ConeDetection::*YELLOW*)
        ((= const 3) 'qcar_navigation::ConeDetection::*ORANGE*)
        (t nil)))

(defclass qcar_navigation::ConeDetection
  :super ros::object
  :slots (_header _colour _direction _range _position _position_covariance _bbox_x _bbox_y _bbox_w _bbox_h _pixel_count ))

(defmethod qcar_navigation::ConeDetection
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:colour __colour) 0)
    ((:direction __direction) (instance geometry_msgs::Vector3 :init))
    ((:range __range) 0.0)
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:position_covariance __position_covariance) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:bbox_x __bbox_x) 0)
    ((:bbox_y __bbox_y) 0)
    ((:bbox_w __bbox_w) 0)
    ((:bbox_h __bbox_h) 0)
    ((:pixel_count __pixel_count) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _colour (round __colour))
   (setq _direction __direction)
   (setq _range (float __range))
   (setq _position __position)
   (setq _position_covariance __position_covariance)
   (setq _bbox_x (round __bbox_x))
   (setq _bbox_y (round __bbox_y))
   (setq _bbox_w (round __bbox_w))
   (setq _bbox_h (round __bbox_h))
   (setq _pixel_count (round __pixel_count))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:colour
   (&optional __colour)
   (if __colour (setq _colour __colour)) _colour)
  (:direction
   (&rest __direction)
   (if (keywordp (car __direction))
       (send* _direction __direction)
     (progn
       (if __direction (setq _direction (car __direction)))
       _direction)))
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:position_covariance
   (&optional __position_covariance)
   (if __position_covariance (setq _position_covariance __position_covariance)) _position_covariance)
  (:bbox_x
   (&optional __bbox_x)
   (if __bbox_x (setq _bbox_x __bbox_x)) _bbox_x)
  (:bbox_y
   (&optional __bbox_y)
   (if __bbox_y (setq _bbox_y __bbox_y)) _bbox_y)
  (:bbox_w
   (&optional __bbox_w)
   (if __bbox_w (setq _bbox_w __bbox_w)) _bbox_w)
  (:bbox_h
   (&optional __bbox_h)
   (if __bbox_h (setq _bbox_h __bbox_h)) _bbox_h)
  (:pixel_count
   (&optional __pixel_count)
   (if __pixel_count (setq _pixel_count __pixel_count)) _pixel_count)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _colour
    1
    ;; geometry_msgs/Vector3 _direction
    (send _direction :serialization-length)
    ;; float64 _range
    8
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; float64[4] _position_covariance
    (* 8    4)
    ;; int32 _bbox_x
    4
    ;; int32 _bbox_y
    4
    ;; int32 _bbox_w
    4
    ;; int32 _bbox_h
    4
    ;; int32 _pixel_count
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _colour
       (write-byte _colour s)
     ;; geometry_msgs/Vector3 _direction
       (send _direction :serialize s)
     ;; float64 _range
       (sys::poke _range (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; float64[4] _position_covariance
     (dotimes (i 4)
       (sys::poke (elt _position_covariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32 _bbox_x
       (write-long _bbox_x s)
     ;; int32 _bbox_y
       (write-long _bbox_y s)
     ;; int32 _bbox_w
       (write-long _bbox_w s)
     ;; int32 _bbox_h
       (write-long _bbox_h s)
     ;; int32 _pixel_count
       (write-long _pixel_count s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _colour
     (setq _colour (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Vector3 _direction
     (send _direction :deserialize buf ptr-) (incf ptr- (send _direction :serialization-length))
   ;; float64 _range
     (setq _range (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; float64[4] _position_covariance
   (dotimes (i (length _position_covariance))
     (setf (elt _position_covariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; int32 _bbox_x
     (setq _bbox_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _bbox_y
     (setq _bbox_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _bbox_w
     (setq _bbox_w (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _bbox_h
     (setq _bbox_h (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _pixel_count
     (setq _pixel_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get qcar_navigation::ConeDetection :md5sum-) "41df40de730901088a49c65ee720b31e")
(setf (get qcar_navigation::ConeDetection :datatype-) "qcar_navigation/ConeDetection")
(setf (get qcar_navigation::ConeDetection :definition-)
      "std_msgs/Header header

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



(provide :qcar_navigation/ConeDetection "41df40de730901088a49c65ee720b31e")


