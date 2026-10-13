# EkfState.msg — EKF state + covariance for r(1) and r(4).
Header header
float64[] x                       # first 4 elements: [X; Y; yaw; v]
float64[] P                       # row-major, length = len(x)^2
float64   last_nis                # NaN if no update this step
bool      updated_this_step
