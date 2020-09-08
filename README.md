# Simulator for Selfie

## Structure of simulator
- worlds - maps, their resources and launches preparing them (optionally)
- vehicles - car models, config files for them and launches used for spawning them
- scripts - scripts used as interfaces between simulation and defalt ROS topics used in project and other script-based tools
- launches - basic launches for different scenarios


## Simulation interface
(Based on STM32 Bridge)

### Subscribed topics
#### TODO Update docs about new interfaces used only by simulator (like drive-mode buttons handling etc)

 - `drive/out` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html)) - Steering commands to be applied.

 - `drive/manual` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html)) - Manual steering commands from human

### Published topics

 - `imu` ([sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html)) - Data stream from IMU.

 - `stm32/speed` ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html)) - Linear velocity magnitude at the center of rear axle, as calculated from encoder data (in m/s).

 - `distance` ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html)) - Distance covered by the car.

 - `/image_rect` ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)) - Image from camera

### Published services

 - `ackerman_steering_mode` ([std_srv/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html)) - Service used to set ackerman steering mode (front and back axis move reversely)

 - `parallel_steering_mode` ([std_srv/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html)) - Service used to set parallel steering mode (front and back axis move in the same direction)

 - `front_axis_steering_mode` ([std_srv/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html)) - Service used to set front axis steering mode (only front axis moves)