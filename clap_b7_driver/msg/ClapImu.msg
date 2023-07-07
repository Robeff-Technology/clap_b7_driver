# ADIS16470 IMU

#0x00000001 Alarm Status Flag
#0x00000002 reserved
#0x00000004 reserved
#0x00000008 SPI Communication Error                         0 = Passed, 1 = Failed
#0x00000010 Sensor Over-Range                               0 = Passed, 1 = One of more sensors over-ranged
#0x00000020 Initial Self Test Failure                       0 = Passed, 1= Failed
#0x00000040 Flash Memory Failure                            0 = Passed, 1 = Failed
#0x00000080 Processing Overrun                              0 = Passed, 1 = Failed
#0x00000100 Self Test Failure – X-axis gyro                 0 = Passed, 1 = Failed
#0x00000200 Self Test Failure – Y-axis gyro                 0 = Passed, 1 = Failed
#0x00000400 Self Test Failure – Z-axis gyro                 0 = Passed, 1 = Failed
#0x00000800 Self Test Failure – X-axis accelerometer        0 = Passed, 1 = Failed
#0x00001000 Self Test Failure – Y-axis accelerometer        0 = Passed, 1 = Failed
#0x00002000 Self Test Failure – Z-axis accelerometer        0 = Passed, 1 = Failed
uint32 imu_status

float32 imu_temperature

float32 z_accel_output

float32 y_accel_output

float32 x_accel_output

float32 z_gyro_output

float32 y_gyro_output

float32 x_gyro_output
