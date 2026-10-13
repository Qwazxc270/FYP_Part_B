# ConeArray.msg — detections from RGBD or LiDAR cone pipeline.
Header header
float32[] range
float32[] bearing
float32[] x_body
float32[] y_body
uint8[]   class_id                # 1=yellow (outer), 2=blue (inner)
float32[] confidence
