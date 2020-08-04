#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped


class SimulatorBridge:
    '''Class providing interface between model in simulation and autonomy'''

    def __init__(self):
        rospy.init_node("simulation_bridge", anonymous=True)

        # Init publishers
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
        pass

    pass


if __name__ == '__main__':

    pass
