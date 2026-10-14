std_msgs/Header header
uint8 UNINITIALIZED=0
uint8 CONVERGING=1
uint8 OK=2
uint8 DEGRADED=3                   # running without absolute fixes for too long
uint8 LOST=4
uint8 state
float32 position_std_m             # 1-sigma position uncertainty
float32 yaw_std_rad
float32 seconds_since_absolute_fix
