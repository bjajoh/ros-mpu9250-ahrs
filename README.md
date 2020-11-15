# ros-mpu9250-ahrs
This package is a simple ROS driver with IMU + Magnetometer fusion for the Raspberry Pi in use with the MPU9250.

## Installation
This ROS Package is tested with the Raspberry Pi4 and ROS noetic.

### Install the MPU9250 Python Package
https://github.com/Intelligent-Vehicle-Perception/MPU-9250-Sensors-Data-Collect

### Run the mpu9250 + fusion node
The following launch file automatically calibrates the MPU9250 on startup and starts publishing filtered values.
```
roslaunch mpu9250-driver mpu9250.launch
```
