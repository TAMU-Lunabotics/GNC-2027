std_msgs/Header header
uint8 IDLE=0                       # no goal
uint8 PLANNING=1
uint8 SUCCEEDED=2                  # /plan holds a valid path to the current goal
uint8 FAILED=3                     # no path found; see message
uint8 state
string message                     # human-readable reason, e.g. "goal inside obstacle"
float32 path_length_m
float32 planning_time_s
