# 0 SOLUTION_COMPUTED
# 1 INSUFFICIENT_OBS
# 2 NO_CONVERGENCE
# 3 COV_TRACE
uint16 status


#clockwise angle between True North and baseline vector from master antenna to the slave antenna
#True north heading angle(0 to 360 degree)
float32 heading

#standard deviation of calculation heading angle
float32 std_dev_heading

#pitch angle(-90 to +90 degree)
float32 pitch

#standard deviation of calculation pitch angle
float32 std_dev_pitch

#distance between master antenna and slave antenna in meter
float32 baseline_len