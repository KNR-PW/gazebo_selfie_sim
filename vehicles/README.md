
## Directories
- sensors - models of sensors used by vehicle
- models - models of cars
- models/{car_name} - directory with described models of cars
- launches/spawn_car - spawn a model specified by the model_name argument



## Interfaces
Each car model should use these topics (not all of them, but these are naming conventions for typical car topics).
They will be later used by scripts providing interfaces to other ROS nodes.

### Naming conventions for car's topics:

Input:
- `/vehicle/{left,right}_{rear,front}_wheel_velocity_controller/command` Float64 - used for steering rotation speed of car wheels
- `/vehicle/{left,right}_{rear,front}_steering_hinge_position_controller/command` Float64 - used for steering position of car wheels (not always 4 wheels must be controllable)
- `/{left,right}_indicator` Bool - specifies whether to blink the turn indicator

Output:
- `/image_rect` sensor_msgs/Image - Image form camera
- `/scan` sensor_msgs/LaserScan - Message published by LIDAR 
- `/imu` nav_msgs/Imu - Message published by Imu 

### Parameters of car:
- `vehicle/wheel_radius` float - radius of wheels (in meters)
- `vehicle/wheel_width` float - width of wheels (in meters)
- `vehicle/wheel_mass` float - mass of wheels (in kg)

- `vehicle/front_wheel_x` float - position of the front wheels on the x axis(in meters)
- `vehicle/rear_wheel_x` float - position of the rear wheels on the x axis(in meters)
- `vehicle/wheel_y` float - absolute value of position of the wheels on the y axis(in meters)

- `vehicle/hinge_radius` float - radius of the hinge sphere (in meters)
- `vehicle/hinge_mass` float - mass of the hinge (in kg)

- `vehicle/chassis_x` float - length of the chassis in the x dimention (in meters)
- `vehicle/chassis_y` float - length of the chassis in the y dimention (in meters)
- `vehicle/chassis_z` float - length of the chassis in the z dimention (in meters)
- `vehicle/chassis_mass` float - mass of the chassis (in kg)



- `vehicle/indicator_front_x` float - position of the front indicator on the x axis (in meters)
- `vehicle/indicator_back_x` float - position of the back indicator on the x axis (in meters)
- `vehicle/indicator_y` float - absolute value of the position of the indicator on the y axis (in meters)
- `vehicle/indicator_z` float - the position of the indicator on the z axis (in meters)
- `vehicle/indicator_width` float - width of the indicator cylinder (in meters)
- `vehicle/indicator_radius` float - radius of the indicator cylinder (in meters)
