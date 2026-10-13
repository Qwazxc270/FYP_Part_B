# FaultFlags.msg — human-readable flags (mirrors init_faults_struct).
Header header
bool camera_timeout
bool lidar_timeout
bool cones_cam_timeout
bool cones_lidar_timeout
bool cones_both_timeout
bool imu_timeout
bool encoder_timeout
bool steering_fault
bool drive_fault
bool ekf_divergence
bool cte_exceeded
bool speed_deviation
bool mpc_overrun
bool obstacle_detected
bool battery_low
bool overcurrent
