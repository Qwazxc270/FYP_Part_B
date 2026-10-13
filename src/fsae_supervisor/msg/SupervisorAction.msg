# SupervisorAction.msg — Stage-4 output. Subscribed to by every
# downstream controller that must honour the supervisor's decision.
Header header
uint8   mode_coarse              # 1=Nominal, 2=Degraded, 3=Critical
uint8   mode_fine                # 1..6, fine mode index
string  mode_name
string  mode_name_fine
float32 speed_scale              # multiplicative factor in [0,1]
bool    estop
bool    adjust_slam              # request to down-weight camera detections in ekf_slam_node (/odometry/filtered producer), NOT nav_filter_node
string  fault_isolation          # {none, Perception, Proprioception, Compute, Actuation, Critical (multiple)}
string  override_cause           # empty if IMM-driven; else hard-override cause(s)
float32 severity                 # 10*P(Crit) + 5*P(Deg)
float32[6] mu_fine
float32[3] mu_coarse
