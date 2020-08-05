
## Directories
- sensors - models of sensors used by vehicle
- models - models of cars
- models/{car_name} - directory with described models of cars


## Interfaces
Each car model should use these topics (not all of them, but these are naming conventions for typical car topics).
They will be later used by scripts providing interfaces to other ROS nodes.

### Naming conventions for car's topics:

Input:
- `/vehicle/{left,right}_{rear,front}_wheel_velocity_controller/command` Float64 - used for steering rotation speed of car wheels
- `/vehicle/{left,right}_{rear,front}_steering_hinge_position_controller/command` Float64 - used for steering position of car wheels (not always 4 wheels must be controllable)

Output:
- `/image_rect` sensor_msgs/Image - Image form camera
- `/scan` sensor_msgs/LaserScan - Message published by LIDAR 

### Parameters of car:
- `vehicle/wheel_radius` type-TODO - radius of wheels (in meters)