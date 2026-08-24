;; Auto-generated. Do not edit!


(when (boundp 'qcar_guidance::TrajectoryMessage)
  (if (not (find-package "QCAR_GUIDANCE"))
    (make-package "QCAR_GUIDANCE"))
  (shadow 'TrajectoryMessage (find-package "QCAR_GUIDANCE")))
(unless (find-package "QCAR_GUIDANCE::TRAJECTORYMESSAGE")
  (make-package "QCAR_GUIDANCE::TRAJECTORYMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryMessage.msg.html


(defclass qcar_guidance::TrajectoryMessage
  :super ros::object
  :slots (_waypoint_times _waypoint_x _waypoint_y _velocity ))

(defmethod qcar_guidance::TrajectoryMessage
  (:init
   (&key
    ((:waypoint_times __waypoint_times) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:waypoint_x __waypoint_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:waypoint_y __waypoint_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocity __velocity) 0.0)
    )
   (send-super :init)
   (setq _waypoint_times __waypoint_times)
   (setq _waypoint_x __waypoint_x)
   (setq _waypoint_y __waypoint_y)
   (setq _velocity (float __velocity))
   self)
  (:waypoint_times
   (&optional __waypoint_times)
   (if __waypoint_times (setq _waypoint_times __waypoint_times)) _waypoint_times)
  (:waypoint_x
   (&optional __waypoint_x)
   (if __waypoint_x (setq _waypoint_x __waypoint_x)) _waypoint_x)
  (:waypoint_y
   (&optional __waypoint_y)
   (if __waypoint_y (setq _waypoint_y __waypoint_y)) _waypoint_y)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:serialization-length
   ()
   (+
    ;; float64[] _waypoint_times
    (* 8    (length _waypoint_times)) 4
    ;; float64[] _waypoint_x
    (* 8    (length _waypoint_x)) 4
    ;; float64[] _waypoint_y
    (* 8    (length _waypoint_y)) 4
    ;; float64 _velocity
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _waypoint_times
     (write-long (length _waypoint_times) s)
     (dotimes (i (length _waypoint_times))
       (sys::poke (elt _waypoint_times i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _waypoint_x
     (write-long (length _waypoint_x) s)
     (dotimes (i (length _waypoint_x))
       (sys::poke (elt _waypoint_x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _waypoint_y
     (write-long (length _waypoint_y) s)
     (dotimes (i (length _waypoint_y))
       (sys::poke (elt _waypoint_y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _waypoint_times
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoint_times (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _waypoint_times i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _waypoint_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoint_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _waypoint_x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _waypoint_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoint_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _waypoint_y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _velocity
     (setq _velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get qcar_guidance::TrajectoryMessage :md5sum-) "3970ebd2c3a27d3351680b715e22ec1f")
(setf (get qcar_guidance::TrajectoryMessage :datatype-) "qcar_guidance/TrajectoryMessage")
(setf (get qcar_guidance::TrajectoryMessage :definition-)
      "float64[] waypoint_times
float64[] waypoint_x
float64[] waypoint_y
float64 velocity
")



(provide :qcar_guidance/TrajectoryMessage "3970ebd2c3a27d3351680b715e22ec1f")


