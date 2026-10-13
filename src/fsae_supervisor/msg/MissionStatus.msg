# MissionStatus.msg — FSM state output.
Header header
uint8  STATE_WAITING=1
uint8  STATE_RUNNING=2
uint8  STATE_FINISHED=3
uint8  STATE_ESTOPPED=4
uint8  state
string state_name
string mode                       # "lap" | "time"
uint32 current_lap
uint32 lap_count_target
float32 run_time_s
float32 time_limit_s
float32 target_speed_cmd
float32 last_lap_time_s
