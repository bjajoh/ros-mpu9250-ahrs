# ros-mpu9250-ahrs
ROS driver with imu + magnetometer fusion for the Raspberry Pi in use with the MPU9250.
It calibrates all sensors during the start and starts publishing all filtered states once the calibration is finished.

## Installation
This ROS Package is tested with the Raspberry Pi4 and ROS noetic

### Install the MPU9250 Python Package
https://github.com/Intelligent-Vehicle-Perception/MPU-9250-Sensors-Data-Collect

### Run the mpu9250 + fusion node
roslaunch mpu9250-driver mpu9250.launch
