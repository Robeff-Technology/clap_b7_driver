std_msgs/Header header

#0           INS_INACTIVE
#1           INS_ALIGNING
#2           INS_HIGH_VARIANCE
#3           INS_SOLUTION_GOOD
#6           INS_SOLUTION_FREE
#7           INS_ALIGNMENT_COMPLETE
uint32 ins_status

#1-51    Reserved
#52      INS                 INS position solution
#53      INS_PSRSP           Intergrated solution of INS and single point positioning – no DGPS correction
#54      INS_PSRDIFF         Intergrated solution of INS and DGPS
#55      INS_RTKFLOAT        Intergrated solution of INS and the RTK floating
#56      INS_RTKFIXED        Intergrated solution of INS and the RTK fixed
#57-74   Reserved
uint32 pos_type

#Latitude (WGS84) (degrees)
float64 latitude

#Longitude (WGS84) (degrees)
float64 longitude

#Height above mean sea level (m)
float64 height

#Undulation, the distance between the geoid and the WGS84 ellipsoid.
#The geoid is positive when it is above the ellipsoid, otherwise, it is negative (m)
float32 undulation

#Velocity in a northbound direction (a negative value implies a southerly direction) (m/s)
float64 north_velocity

#Velocity in an eastbound direction (a negative value implies a westbound direction) (m/s)
float64 east_velocity

#Velocity in up direction (m/s)
float64 up_velocity

#Right-handed rotation from local
#level around y-axis in (degrees)
float64 roll

#Right-handed rotation from local
#level around x-axis in (degrees)
float64 pitch

#Left-handed rotation around z-axis in (degrees) clockwise from north.
#This is the inertial azimuth calculated from the IMU gyros and the SPAN filters.
float64 azimuth

#Latitude standard deviation (m)
float32 std_dev_latitude

#Longitude standard deviation (m)
float32 std_dev_longitude

#Height standard deviation (m)
float32 std_dev_height

#North velocity standard deviation (m/s)
float32 std_dev_north_velocity

#East velocity standard deviation (m/s)
float32 std_dev_east_velocity

#Up velocity standard deviation (m/s)
float32 std_dev_up_velocity

#Roll standard deviation (degrees)
float32 std_dev_roll

#Pitch standard deviation (degrees)
float32 std_dev_pitch

#Azimuth standard deviation (degrees)
float32 std_dev_azimuth

#0x00000001      Position update is used in the last update epoch
#0x00000002      Phase update is used in the last update epoch
#0x00000004      ZUPT update is used in the last update epoch
#0x00000008      Odometer update is used in the last update epoch
uint32 extended_solution_stat

#Elapsed time since the last ZUPT or position update (second)
uint16 time_since_update
