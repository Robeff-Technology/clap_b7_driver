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

2.  Build Tools: Ensure you have the necessary build tools and dependencies installed on your system.

### Install the Clap-B7 ROS2 Driver

1.  Clone the Repository: If your Clap-B7 ROS2 driver is hosted on a version control system like Git, clone the repository into your ROS2 workspace's source directory:
```
    cd /path/to/your/ros2_workspace/src
    git clone https://github.com/Robeff-Technology/clap_b7_driver.git 
```     
-   Build the Workspace: Navigate to your ROS2 workspace and build the packages:
    ```
    cd /path/to/your/ros2_workspace
    colcon build --packages select clap_b7_driver
    ```  
-   Source the Workspace: Source your ROS2 workspace to make the newly built Clap-B7 driver node available:
    ```
    source /path/to/your/ros2_workspace/install/setup.bash 
    ```


### Usage

1.  Launch the Clap-B7 Driver Node: To start the Clap-B7 driver node, use the provided launch file. The launch file should be located in the package's `launch` directory. Run the following command:
```
ros2 launch clap_b7_driver clap_b7_driver.launch.py` 
 ```
The launch file will start the driver node and configure it based on the default parameters.

2.  Verify Data: After launching the driver, check if the GNSS and INS data are being published to the correct topics using tools like `ros2 topic echo`.


### Customizing Configuration (Optional)

If you wish to customize the behavior of the GNSS/INS driver by adjusting its parameters, you can use a `.yaml` configuration file. By default, the launch file might already include a reference to the configuration file.

1.  Locate the Configuration File: The `.yaml` configuration file should be located in the package's `config` directory.

2.  Customize Parameters: Edit the `.yaml` configuration file to modify the driver's behavior according to your requirements. You can adjust parameters such as communication settings, sensor fusion parameters, and output topics.

3.  Launch the Driver with Custom Configuration: After making the necessary changes to the configuration file, launch the driver node with the updated configuration using the following command:

`ros2 launch clap_b7_driver clap_b7_driver.launch.py config:=/path/to/your/config.yaml`

Replace `/path/to/your/config.yaml` with the actual path to your edited configuration file.


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