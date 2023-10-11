# Unicorecomm Clap-B7 ROS2 Driver

## Overview

The Clap-B7 ROS2 driver is a software component designed to interface with a Global Navigation Satellite System (GNSS) module combined with an Inertial Navigation System (INS). The driver enables ROS2-based applications to access and utilize the raw GNSS and INS data for localization, navigation, and other related tasks.

#### Author: [Robeff Technology](https://www.robeff.com)
#### Maintainer : [Robeff Technology](mailto:support@robeff.com)
## Features

-   **ROS2 Compatibility**: The driver is fully compatible with the ROS2 (Robot Operating System 2) ecosystem, allowing seamless integration with other ROS2 nodes and packages.

-   **GNSS Data**: The driver reads raw data from the GNSS module, including satellite positions, timestamps, position, velocity, and other relevant information.

-   **INS Data**: It retrieves data from the Inertial Navigation System, such as roll, pitch, and yaw angles.

-   **6-DOF IMU Data**: The Clap-B7 has a built-in [ADIS16470](https://www.analog.com/media/en/technical-documentation/data-sheets/adis16470.pdf) 6-DOF IMU (Inertial Measurement Unit) that provides data from the accelerometers and gyroscopes.
-   **Fused Output**: The Clap-B7 GNSS module can fuse GNSS and INS data using sensor fusion techniques (e.g., Extended Kalman Filter) to provide an accurate and robust localization estimate.

-   **Configurable Parameters**: Various parameters can be configured to adapt the driver to different GNSS/INS modules and user requirements.

## Installation

To install the  Clap-B7 ROS2 driver, follow these steps:

### Prerequisites

Before proceeding with the installation, ensure you have the following prerequisites:

1.  ROS2 Humble: Make sure you have a working ROS2 Humble installation. If you don't have ROS2 Humble installed, you can follow the official installation instructions: [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation.html).

    **Dependencies**: The driver includes additional dependencies:
    -   [mavros_msgs](https://github.com/mavlink/mavros)
    ```
       sudo apt install ros-<distro>-mavros-msgs
    ```
    - [GeographicLib](https://geographiclib.sourceforge.io/html/)
    ```
       sudo apt install libgeographic-dev
    ```

2.  Build Tools: Ensure you have the necessary build tools and dependencies installed on your system.

### Install the Clap-B7 ROS2 Driver

1.  Clone the Repository:clone the repository into your autoware workspace's source directory:
```
    cd /path_to_your_ros2_workspace/src
    git clone https://github.com/Robeff-Technology/clap_b7_driver.git 
```     
-   Build the Workspace: Navigate to your autoware workspace and build the packages:
    ```
    cd /path_to_your_ros2_workspace
    colcon build --packages select clap_b7_driver
    ```  
-   Source the Workspace: Source your autoware workspace to make the newly built Clap-B7 driver node available:
    ```
    source /path_to_your_ros2_workspace/install/setup.bash 
    ```


### Usage
1. Launch the Clap-B7 Configuration Node: To start the Clap-B7 configuration node, Run the following command:
```
ros2 launch clap_b7_driver config_clap_b7.launch.py 
```
2.  Launch the Clap-B7 Driver Node: To start the Clap-B7 driver node, use the provided launch file. The launch file should be located in the package's `launch` directory. Run the following command:
```
ros2 launch clap_b7_driver clap_b7_driver.launch.py 
 ```
The launch file will start the driver node and configure it based on the default parameters.

3.  Verify Data: After launching the driver, check if the GNSS and INS data are being published to the correct topics using tools like `ros2 topic echo`.

### Customizing Configuration (Optional)

If you wish to customize the behavior of the GNSS/INS driver by adjusting its parameters, you can use a `.yaml` configuration file. By default, the launch file might already include a reference to the configuration file.

1.  Locate the Configuration File: The `.yaml` configuration file should be located in the package's `config` directory.

2.  Customize Parameters: Edit the `.yaml` configuration file to modify the driver's behavior according to your requirements. You can adjust parameters such as communication settings, ins settings and output topics.

3.  Launch the Driver with Custom Configuration: After making the necessary changes to the configuration file, launch the driver node with the updated configuration using the following command:

```
ros2 launch clap_b7_driver clap_b7_driver.launch.py
```


### Troubleshooting

-   **Build Errors**: If you encounter build errors during the `colcon build` process, ensure that all dependencies are properly installed and sourced.

-   **Permissions**: If you're having trouble accessing the GNSS/INS module (e.g., /dev/ttyUSB0), make sure the user has the necessary permissions to access the serial port or other communication interfaces.

-   **Driver-specific Issues**: For driver-specific issues or questions, refer to the documentation provided by the developer or contact the developer directly using the provided contact information.

## Custom Messages

The Clap-B7 ROS2 driver uses custom messages to represent specific data relevant to your application. Below is a list of custom messages and their explanations:

### `ClapECEF`
-   **Description**: Position and velocity in ECEF.

### `ClapGpsPos`
-   **Description**: The best available GNSS position (without INS) computed by the receiver.

### `ClapGpsVel`
-   **Description**: The best available GNSS velocity (without INS) computed by the receiver. In addition, it reports velocity status indicators, which is of great use for indicating the validity of corresponding data. The speed measurement sometimes causes related latency.

### `ClapHeading`
-   **Description**: Heading information of the receiver in motion. Heading refers to the clockwise angle between True North and the baseline vector from the master antenna to the slave antenna.

### `ClapImu`
-   **Description**: IMU status indicator and the measurements from the accelerometers and gyros with respect to the IMU enclosure frame.

### `ClapIns`
-   **Description**: Integrated navigation results and deviations.

### `ClapWheelOdom`
-   **Description**: Odometry messages according to the wheel speed to clap.

## Standard Messages

### `imu/Temp` [sensor_msgs/Temperature](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Temperature.html)
- Temperature of the IMU. Requires  `ClapImu`.

### `TwistWithCovarianceStamped `[geometry_msgs/TwistWithCovarianceStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html)
- Velocity of the vehicle. Requires `ClapGpsVel`, `ClapHeading`, `ClapImu`.
- /raw/ecef_twist : Velocity of the vehicle in ECEF. Requires `ClapECEF`.

### `NavSatFix` [sensor_msgs/NavSatFix](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html)
- Position of the vehicle. Requires `ClapIns`.
- /raw/nav_sat_fix: Position of the vehicle without EKF. Requires `ClapGpsPos`.

### `Imu` [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)
- IMU data. Requires `ClapImu`, `ClapIns`.
- raw/imu : IMU data without EKF. Requires `ClapImu`.

### `Odometry` [nav_msgs/Odometry](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html)
- Odometry data. Requires `ClapIns`.

## Subscribed Topic
### `RTCM` [mavros_msgs::msg::RTCM](https://docs.ros.org/en/api/mavros_msgs/html/msg/RTCM.html)
- RTCM data for RTK. For more information [ntrip_client.](https://github.com/Robeff-Technology/ntrip_client)

# Clap - B7 Configuration
If user wants to change message period, ins configuration and pps configuration, can change the configuration file.
The configuration file is located in the package's `config` directory.
The configuration file is in the `.yaml` format and named with `config_clap_b7.param.yaml`. The following sections describe the different parameters that can be configured in the file.

## `serial_config`

- `port`: The name of the serial port where the device is connected.
- `baudrate`: The baud rate for serial communication.
- `clap_port`: Indicates which port the "clap-b7" is connected to (e.g., port 1, port 2, port 3).

## `port1_config`, `port2_config`, `port3_config`

These sections configure different ports, likely for various functionalities or devices. Common parameters include:

- `baudrate`: The baud rate for the respective port.
- `rawimu_period`: The period for rawimu messages.
- `inspvax_period`: The period for inspvax messages.
- `uniheading_period`: The period for uniheading messages.
- `bestgnsspos_period`: The period for bestgnsspos messages.
- `bestgnssvel_period`: The period for bestgnssvel messages.
- `ecef_period`: The period for ecef messages.
- `wheel_speed_period`: The period for wheel speed messages.
- `gprmc`: Boolean value to indicate whether to send gprmc messages.

## ins_config
- `enable`: Indicates whether the Inertial Navigation System (INS) is enabled (true) or disabled (false).
- `timeout`: Sets the duration for INS output when losing GNSS signals (in seconds).
- `align_velocity_threshold`: Specifies the velocity threshold for INS alignment (in m/s).
- `lever_arm_master` and `lever_arm_slave`: Lever arm settings for the master and slave antennas relative to the IMU.
- `lever_arm_master_error` and `lever_arm_slave_error`: Lever arm error settings for the antennas.
- `imu_position_offset`: Position offsets for the IMU.

## `pps_config`

- `enable`: Indicates whether the Pulse Per Second (PPS) signal is enabled (true) or disabled (false).
- `mode`: Sets the mode for PPS output.
- `polarity`: Specifies the polarity of the PPS signal.
- `width`: Sets the pulse width of the PPS signal (in microseconds).
- `period`: Sets the period of the PPS signal (in milliseconds).

These configurations allow you to customize the communication settings and behavior of the connected device within the ROS2 environment. You can adjust parameters such as baud rates, message publication periods, INS settings, and PPS settings to suit your specific requirements.