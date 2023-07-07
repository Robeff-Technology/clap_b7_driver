#degree
float64 latitude
float64 longitude
float64 altitude

#the distance between the geoid and the WGS84 ellipsoid. The geoid is positive when it is above the
#ellipsoid, otherwise, it is negative [m]
float32 undulation

#the standard deviation of the latitude, longitude and altitude [m]
float32 std_dev_latitude
float32 std_dev_longitude
float32 std_dev_altitude

uint8 number_of_satellite_tracked
uint8 number_of_satellite_used

#the age of the differential correction [s]
float32 diff_age