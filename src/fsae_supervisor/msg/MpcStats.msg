# MpcStats.msg — published each MPC step, consumed by the supervisor.
Header header
float32 compute_time_ms
float32 budget_ms
bool    saturated
float32 delta_cmd
float32 v_ref
float32 cte
