std_msgs/Header header
uint8 IDLE=0                       # no path
uint8 FOLLOWING=1
uint8 GOAL_REACHED=2
uint8 STOPPED=3                    # stopped for a reason other than success; see message
uint8 state
string message                     # e.g. "no localization for 0.5 s", "path empty"
float32 cross_track_error_m        # lateral distance from the path
float32 distance_to_goal_m
