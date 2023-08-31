#counts per revolution
uint16 ticks_per_rev

#counts/s
uint16 wheel_speed

#counts/s
float32 f_wheel_speed

# 0 -> stop 1 -> forward 2 -> backward 3->unknown direction
uint32 direction

#total count
int32 cumulative_ticks