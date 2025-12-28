# ros2_imu_serial

A **ROS 2 package** for reading 6-axis **IMU (Inertial Measurement Unit) (MPU6050) data over a serial interface** and publishing it as standard ROS 2 messages. This project is designed to make it easy to integrate low-cost or custom IMU hardware into a ROS 2-based robotics system.

## Features

- Reads IMU data from a serial port
- Publishes data as standard `sensor_msgs/msg/Imu`
- Configurable serial port and baud rate
- Lightweight and easy to integrate into existing ROS 2 systems
- Suitable for embedded IMUs, microcontroller-based sensors, and prototyping

## 📦 Package Overview
v
This package provides:

- A ROS 2 node that:
  - Opens a serial connection to an IMU device
  - Parses incoming sensor data
  - Publishes orientation, angular velocity, linear acceleration, and Euler angles

## 🔧 Requirements

- **ROS 2** (tested with common distributions such as Humble / Iron)
- **Linux** (recommended for serial device support)
- IMU device that outputs serial data in a supported format
- Access permissions to the serial device (e.g. `/dev/ttyUSB0`)

## 📥 Installation

Clone the repository into your ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/SlyPredator/ros2_imu_serial.git
```

Build the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

Run the IMU serial node:

```bash
ros2 launch ros2_imu_serial imu_publisher.launch.py
```

## 📡 Published Topics

Topics published by this package include:

- `/imu/data_raw` (`sensor_msgs/msg/Imu`)
- `/imu/euler` (`sensor_msgs/msg/Imu`)

You can inspect the data using:

```bash
ros2 topic echo /imu/data_raw
ros2 topic echo /imu/euler
```

## Parameters

| Parameter       | Type   | Description                                | Default        |
| --------------- | ------ | ------------------------------------------ | -------------- |
| `serial_port`   | string | Serial device path                         | `/dev/ttyUSB0` |
| `baudrate`      | int    | Serial communication baud rate             | `115200`       |
| `frame_id`      | string | TF frame for the IMU data                  | `imu_link`     |
| `publish_rate`  | float  | Rate of topic publishing in ROS2           | `50`           |
| `use_madgwick`  | bool   | Whether or not to apply Madgwick Filtering | `true`         |
| `madgwick_beta` | float  | Madgwick Filtering beta value              | `0.1`          |
| `publish_euler` | bool   | Whether or not to publish Euler angles     | `true`         |

## Permissions

If you encounter permission errors accessing the serial port:

```bash
sudo usermod -a -G dialout $USER
newgrp dialout
```

## Notes

- IMU data format and parsing depend on your specific hardware
- Sensor calibration may be required for accurate results
- Orientation covariance and filtering are left to downstream nodes (e.g. `robot_localization`)

## License

This project is open-source. Please check the repository for license details.

## Contributing

Contributions, bug reports, and feature requests are welcome!
Feel free to open an issue or submit a pull request.

## Acknowledgements

Built for ROS 2 users who want a simple and flexible way to bring serial IMU data into their robotic systems.

Happy hacking! 🚀
