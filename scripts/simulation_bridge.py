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
        self.wheels_perimeter = rospy.get_param("/wheel_radius")*2*pi

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

        pass

    def run(self):

        pass

    def drive_callback(self, data):
        if self.mode == DriveMode.AUTOMATIC:
            self.load_speeds(data)
            self.load_hinges_positions(data)
            pass
        elif self.mode == DriveMode.SEMI_AUTOMATIC:
            self.load_hinges_positions(data)
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
            self.load_speeds(data)
            pass
        elif self.mode == DriveMode.MANUAL:
            self.load_speeds(data)
            self.load_hinges_positions(data)
            pass
        else:
            rospy.logerr("Wrong value of mode")

        self.move_model()
        pass

    def load_speeds(self, data):
        # TODO
        pass

    def load_hinges_positions(self, data):
        # TODO
        pass

    def move_model(self):
        '''Send data about position of hinges and speed of wheels to model'''
        # TODO
        pass

    pass


if __name__ == '__main__':

    pass
