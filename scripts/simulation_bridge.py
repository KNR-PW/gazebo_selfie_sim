#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

from enum import Enum
from math import pi


class DriveMode(Enum):
    MANUAL = "MANUAL"
    AUTOMATIC = "AUTOMATIC"
    SEMI_AUTOMATIC = "SEMI-AUTOMATIC"
    pass


class SimulatorBridge:
    '''Class providing interface between model in simulation and autonomy'''

    def __init__(self):
        rospy.init_node("simulation_bridge", anonymous=True)
        self.mode = DriveMode.AUTOMATIC

        # Read parameters
        self.wheels_radius = rospy.get_param("/wheel_radius", default=0.03)
        self.wheels_perimeter = self.wheels_radius*2*pi

        # Init Subscribers
        self.sub_manual_drive = rospy.Subscriber(
            "/manual_drive", AckermannDriveStamped, self.manual_drive_callback)
        self.sub_drive = rospy.Subscriber(
            "/drive", AckermannDriveStamped, self.drive_callback)

        # TODO add Subscribers/services for switching mode

        # Init publishers
        self.pub_speed = rospy.Publisher('/speed', Float32, queue_size=1)
        self.pub_distance = rospy.Publisher('/distance', Float32, queue_size=1)

        self.pub_vel_left_rear_wheel = rospy.Publisher(
            '/vehicle/left_rear_wheel_velocity_controller/command', Float64,
            queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher(
            '/vehicle/right_rear_wheel_velocity_controller/command', Float64,
            queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher(
            '/vehicle/left_front_wheel_velocity_controller/command', Float64,
            queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher(
            '/vehicle/right_front_wheel_velocity_controller/command', Float64,
            queue_size=1)
        self.pub_pos_left_front_steering_hinge = rospy.Publisher(
            '/vehicle/left_front_steering_hinge_position_controller/command', Float64,
            queue_size=1)
        self.pub_pos_right_front_steering_hinge = rospy.Publisher(
            '/vehicle/right_front_steering_hinge_position_controller/command', Float64,
            queue_size=1)
        self.pub_pos_left_rear_steering_hinge = rospy.Publisher(
            '/vehicle/left_rear_steering_hinge_position_controller/command', Float64,
            queue_size=1)
        self.pub_pos_right_rear_steering_hinge = rospy.Publisher(
            '/vehicle/right_rear_steering_hinge_position_controller/command', Float64, queue_size=1)

        # Other variables
        self.hinge_angle = 0
        self.wheels_speed = 0  # speed in rad/s
        self.distance = 0
        self.is_timer_measueable = False
        self.time_of_last_measurement = rospy.get_time()

        pass

    def run(self):
        rospy.spin()
        pass

    def drive_callback(self, data):
        if self.mode == DriveMode.AUTOMATIC:
            self.wheels_speed = data.drive.steering_angle
            self.hinge_angle = data.drive.steering_angle

            pass
        elif self.mode == DriveMode.SEMI_AUTOMATIC:
            self.hinge_angle = data.drive.steering_angle

            pass
        elif self.mode == DriveMode.MANUAL:
            return
        else:
            rospy.logerr("Wrong value of mode")

        self.move_model()
        pass

    def manual_drive_callback(self, data):
        if self.mode == DriveMode.AUTOMATIC:
            return
        elif self.mode == DriveMode.SEMI_AUTOMATIC:
            self.wheels_speed = data.drive.steering_angle
            pass
        elif self.mode == DriveMode.MANUAL:
            self.wheels_speed = data.drive.steering_angle
            self.hinge_angle = data.drive.steering_angle

            pass
        else:
            rospy.logerr("Wrong value of mode")

        self.move_model()
        pass

    def move_model(self):
        '''Send data about position of hinges and speed of wheels to model'''

        # TODO add modes for Ackermann etc
        hinge_front_left = -self.hinge_angle
        hinge_front_right = -self.hinge_angle
        hinge_rear_left = 0
        hinge_rear_right = 0

        # Send data
        self.pub_vel_left_rear_wheel.publish(self.wheels_speed)
        self.pub_vel_right_rear_wheel.publish(self.wheels_speed)
        self.pub_vel_left_front_wheel.publish(self.wheels_speed)
        self.pub_vel_right_front_wheel.publish(self.wheels_speed)

        self.pub_pos_left_front_steering_hinge.publish(hinge_front_left)
        self.pub_pos_right_front_steering_hinge.publish(hinge_front_right)
        self.pub_pos_left_rear_steering_hinge.publish(hinge_rear_left)
        self.pub_pos_right_rear_steering_hinge.publish(hinge_rear_right)

        self.send_speed_and_distance()
        pass

    def send_speed_and_distance(self):
        # TODO think about better ways of counting speed and distance
        self.pub_speed.publish(self.wheels_speed*self.wheels_radius)

        if self.is_timer_measueable:
            self.distance = self.distance+self.wheels_speed*self.wheels_radius * \
                (rospy.get_time() - self.time_of_last_measurement)
            self.time_of_last_measurement = rospy.get_time()
        else:
            self.is_timer_measueable = True
            self.time_of_last_measurement = rospy.get_time()

        self.pub_distance.publish(self.distance)
        pass

    pass


if __name__ == '__main__':
    bridge = SimulatorBridge()
    try:
        bridge.run()
    except ValueError as e:
        rospy.logerr("Unexpected error: "+str(e))
    finally:
        rospy.logfatal("Unexpected error occurred, closing node")
    pass
