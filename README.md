# Simulator for Selfie

## Structure of simulator
- worlds - maps, their resources and launches preparing them (optionally)
- vehicles - car models, config files for them and launches used for spawning them
- scripts - scripts used as interfaces between simulation and defalt ROS topics used in project and other script-based tools
- launches - basic launches for different scenarios


## Simulation interface
(Based on STM32 Bridge)

### Subscribed topics

 - `/selfie_in/drive` ([custom_msgs/DriveCommand](./../../Shared/custom_msgs/msg/DriveCommand.msg)) - Steering commands to be applied.

 - `drive/manual` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html)) - Manual steering commands from human

 - `/selfie_in/indicators` ([custom_msgs/Indicators](./../../Shared/custom_msgs/msg/Indicators.msg)) - Activity of right and left indicator

- `/imu` ([sensor_msgs/Imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)) - Data from IMU

- `/vehicle/joint_states` ([sensor_msgs/JointState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html)) - Data about joint state of the car

- `/vehicle/back_distance` ([sensor_msgs/LaserScan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html)) - Data from LIDAR, exists only to publish topic with distance directly behind the car

- `/simulation/switch_state` ([std_msgs/Int8](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int8.html)) - Drive-mode buttons handling


### Published topics
 - `/state/rc` ([std_msgs/Int8](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Int8.html)) - Current state of car

 - `/selfie_out/motion` ([custom_msgs/Motion](./../../Shared/custom_msgs/msg/Motion.msg)) - Topic with custom message that contains: `speed_linear` - linear speed of vehicle, `distance` - distance which vehicle have made so far, `yaw` - Z axis orientation, `speed_yaw` - Z axis angular speed

 - `/image_rect` ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)) - Image from camera

 - `/back_distance` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html)) - Distance behind the car, measured with LIDAR directly behind the car

 - `/sim_left_turn_indicator` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)) - Left indicator boolean for simulation

 - `/sim_right_turn_indicator` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)) - Right indicator boolean for simulation

 - Publishers to send data about position of hinges and speed of wheels to model
    - `/vehicle/left_front_steering_hinge_position_controller/command` [std_msgs/Float64]
    - `/vehicle/left_front_wheel_velocity_controller/command` [std_msgs/Float64]
    - `/vehicle/left_rear_steering_hinge_position_controller/command` [std_msgs/Float64]
    - `/vehicle/left_rear_wheel_velocity_controller/command` [std_msgs/Float64]
    - `/vehicle/right_front_steering_hinge_position_controller/command` [std_msgs/Float64]
    - `/vehicle/right_front_wheel_velocity_controller/command` [std_msgs/Float64]
    - `/vehicle/right_front_wheel_velocity_controller/command` [std_msgs/Float64]
    - `/vehicle/right_rear_wheel_velocity_controller/command` [std_msgs/Float64]

### Published services

 - `ackerman_steering_mode` ([std_srv/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html)) - Service used to set ackerman steering mode (front and back axis move reversely)

 - `parallel_steering_mode` ([std_srv/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html)) - Service used to set parallel steering mode (front and back axis move in the same direction)

 - `front_axis_steering_mode` ([std_srv/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html)) - Service used to set front axis steering mode (only front axis moves)
