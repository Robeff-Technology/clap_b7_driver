# Unicorecomm Clap-B7 ROS2 Driver

## Overview

The Clap-B7 ROS2 driver is a software component designed to interface with a Global Navigation Satellite System (GNSS) module combined with an Inertial Navigation System (INS). The driver enables ROS2-based applications to access and utilize the raw GNSS and INS data for localization, navigation, and other related tasks.

## Features

-   **ROS2 Compatibility**: The driver is fully compatible with the ROS2 (Robot Operating System 2) ecosystem, allowing seamless integration with other ROS2 nodes and packages.

-   **GNSS Data**: The driver reads raw data from the GNSS module, including satellite positions, timestamps, position, velocity, and other relevant information.

-   **INS Data**: It retrieves data from the Inertial Navigation System, such as linear and angular accelerations, roll, pitch, and yaw angles.

-   **Fused Output**: The driver can fuse GNSS and INS data using sensor fusion techniques (e.g., Extended Kalman Filter) to provide an accurate and robust localization estimate.

-   **Configurable Parameters**: Various parameters can be configured to adapt the driver to different GNSS/INS modules and user requirements.

## Installation

To install the  Clap-B7 ROS2 driver, follow these steps:

### Prerequisites

Before proceeding with the installation, ensure you have the following prerequisites:

1.  ROS2 Humble: Make sure you have a working ROS2 Humble installation. If you don't have ROS2 Humble installed, you can follow the official installation instructions: [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation.html).

2.  Autoware.universe: Make sure you have a working autoware.universe installation. If you don't have autoware.universe installed, you can follow the official github repo: [autoware.universe github](https://github.com/autowarefoundation/autoware.universe).
2.  Build Tools: Ensure you have the necessary build tools and dependencies installed on your system.

### Install the Clap-B7 ROS2 Driver

1.  Clone the Repository: If your Clap-B7 ROS2 driver is hosted on a version control system like Git, clone the repository into your autoware workspace's source directory:
```
    cd /path_to_your_autoware_workspace/src/sensor_component/external
    git clone https://github.com/Robeff-Technology/clap_b7_driver.git 
```     
-   Build the Workspace: Navigate to your autoware workspace and build the packages:
    ```
    cd /path_to_your_autoware_workspace
    colcon build --packages select clap_b7_driver
    ```  
-   Source the Workspace: Source your autoware workspace to make the newly built Clap-B7 driver node available:
    ```
    source /path_to_your_autoware_workspace/install/setup.bash 
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

2.  Customize Parameters: Edit the `.yaml` configuration file to modify the driver's behavior according to your requirements. You can adjust parameters such as communication settings, sensor fusion parameters, and output topics.

3.  Launch the Driver with Custom Configuration: After making the necessary changes to the configuration file, launch the driver node with the updated configuration using the following command:

`ros2 launch clap_b7_driver clap_b7_driver.launch.py



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

## Autoware Messages
### `GnssInsOrientationStamped` [autoware_sensing_msgs/GnssInsOrientationStamped](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_sensing_msgs/msg/GnssInsOrientationStamped.msg)
- Orientation of the vehicle. Requires `ClapIns`, `ClapHeading`.

## Subscribed Topic
### `RTCM` [mavros_msgs::msg::RTCM](https://docs.ros.org/en/api/mavros_msgs/html/msg/RTCM.html)
- RTCM data for RTK. For more information [ntrip_client.](https://github.com/Robeff-Technology/ntrip_client)
