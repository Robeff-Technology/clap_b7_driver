std_msgs/Header header

#0 SOL_COMPUTED             Solution computed
#1 INSUFFICIENT_OBS         Insufficient observations
#2 NO_CONVERGENCE           No convergence
#4 COV_TRACE                Covariance trace exceeds maximum (trace > 1000 m)
uint32 sol_status

#0 NONE                      No solution
#1 FIXEDPOS                  Position has been fixed by the FIX position command
#2 FIXEDHEIGHT               Will be supported in the future
#8 DOPPLER_VELOCITY          Velocity computed using instantaneous Doppler
#16 SINGLE                   Single point position
#17 PSRDIFF                  Pseudorange differential solution
#18 SBAS                     SBAS positioning
#32 L1_FLOAT                 Floating L1 ambiguity solution
#33 IONOFREE_FLOAT           Floating ionosphere ambiguity solution
#34 NARROW_FLOAT             Floating narrow-lane ambiguity solution
#48 L1_INTInteger            L1 ambiguity solution
#49 WIDE_INT                 Integer wide-lane ambiguity solution
#50 NARROW_INT               Integer narrow-lane ambiguity solution
#52 INS                      INS position solution
#53 INS_PSRSP                Intergrated solution of INS and single point positioning – no DGPS correction
#54 INS_PSRDIFF              Intergrated solution of INS and DGPS
#55 INS_RTKFLOAT             Intergrated solution of INS and the RTK floating
#56 INS_RTKFIXED             Intergrated solution of INS and the RTK fixed
uint32 vel_type

#The value of the latency calculated from the speed time scale.
#Subtracting it from the epoch time will provide more accurate result.
float32 latency

#differential age,(s)
float32 diff_age

#Horizontal speed over ground, (m/s)
float64 hor_spd

#Actual direction of motion over ground (track over ground) with respect to True North,(deg)
float64 trk_gnd

#Vertical speed, (m/s).
#Where positive values indicate increasing altitude (up) and negative values indicate decreasing altitude (down)
float64 vert_spd
