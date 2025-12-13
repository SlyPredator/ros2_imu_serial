from setuptools import setup
import os
from glob import glob

package_name = "ros2_imu_serial"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),  # Make sure this line exists
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Navneeth Mahadevan",
    maintainer_email="navneethmahadevan@gmail.com",
    description="ROS2 node for reading IMU data from serial port",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_serial_node = ros2_imu_serial.imu_serial_node:main",
        ],
    },
)
