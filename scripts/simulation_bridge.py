#!/usr/bin/env python3
import rospy
import tf
from custom_msgs.msg import DriveCommand
from custom_msgs.msg import Indicators
from custom_msgs.msg import Motion
from std_msgs.msg import Float64
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan

from enum import Enum
from math import pi


class SimulatorBridge:
    '''Class providing interface between model in simulation and autonomy'''

    class DriveMode(object):
        MANUAL = "MANUAL"
        AUTOMATIC = "AUTOMATIC"
        SEMI_AUTOMATIC = "SEMI-AUTOMATIC"

    DRIVE_MODE_NUMBERS = {0: DriveMode.MANUAL,
                          1: DriveMode.SEMI_AUTOMATIC,
                          2: DriveMode.AUTOMATIC}

    def __init__(self):
        rospy.init_node("simulation_bridge", anonymous=True)
        self.mode = self.DriveMode.AUTOMATIC

        # Other variables
        self.hinge_angle_front = 0
        self.hinge_angle_rear = 0
        self.wheels_speed = 0  # speed in rad/s
        self.distance = 0
        self.last_position_of_wheel = 0
        self.imu_yaw = 0
        self.imu_yaw_speed = 0
        # name of joint measured to get speed and distance speed
        self.joint_name = "right_rear_wheel_joint"
        self.is_speed_measurement_started = False

        # Read parameters
        self.wheels_radius = rospy.get_param(
            "/vehicle/wheel_radius", default=0.03)
        self.wheels_perimeter = self.wheels_radius*2*pi

        # Init Subscribers
        self.sub_manual_drive = rospy.Subscriber(
            "/drive/manual", DriveCommand, self.manual_drive_callback)
        self.sub_drive = rospy.Subscriber(
            "/selfie_in/drive", DriveCommand, self.drive_callback)
        self.sub_joints_states = rospy.Subscriber(
            "/vehicle/joint_states", JointState, self.send_speed_and_distance)
        self.sub_indicators = rospy.Subscriber(
            "/selfie_in/indicators", Indicators, self.indicators_callback)
        self.sub_back_dist = rospy.Subscriber(
            "/vehicle/back_distance", LaserScan, self.back_dist_callback)

        # Init Subscribers
        self.sub_drive_mode = rospy.Subscriber(
            "/simulation/switch_state", UInt8, self.change_mode_callback)
        self.sub_imu = rospy.Subscriber(
            '/imu', Imu, self.imu_callback)

        # Init publishers
        self.pub_movement = rospy.Publisher(
            '/selfie_out/motion', Motion, queue_size=1)
        self.pub_drive_mode = rospy.Publisher(
            "/switch_state", UInt8, queue_size=1)
        self.pub_back_dist = rospy.Publisher(
            "/back_distance", Float64, queue_size=1)

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
        self.pub_right_indicator = rospy.Publisher(
            '/sim_right_turn_indicator', Bool, queue_size=1)
        self.pub_left_indicator = rospy.Publisher(
            '/sim_left_turn_indicator', Bool, queue_size=1)

        rospy.loginfo("simulation_bridge initialized")

    def run(self):
        rospy.loginfo("simulation_bridge started")
        rospy.spin()

    def drive_callback(self, data: DriveCommand):
        if self.mode == self.DriveMode.AUTOMATIC:
            self.wheels_speed = data.speed
            self.hinge_angle_front = data.steering_angle_front
            self.hinge_angle_rear = data.steering_angle_rear

        elif self.mode == self.DriveMode.SEMI_AUTOMATIC:
            self.hinge_angle_front = data.steering_angle_front
            self.hinge_angle_rear = data.steering_angle_rear

        elif self.mode == self.DriveMode.MANUAL:
            return
        else:
            rospy.logerr("Wrong value of mode")

        self.move_model()

    def manual_drive_callback(self, data):
        if self.mode == self.DriveMode.AUTOMATIC:
            return
        elif self.mode == self.DriveMode.SEMI_AUTOMATIC:
            self.wheels_speed = data.speed
        elif self.mode == self.DriveMode.MANUAL:
            self.wheels_speed = data.speed
            self.hinge_angle_front = data.steering_angle_front
        else:
            rospy.logerr("Wrong value of mode")

        self.move_model()

    def move_model(self):
        '''Send data about position of hinges and speed of wheels to model'''

        hinge_front_left = self.hinge_angle_front
        hinge_front_right = self.hinge_angle_front
        hinge_rear_left = self.hinge_angle_rear
        hinge_rear_right = self.hinge_angle_rear

        # Send data
        self.pub_vel_left_rear_wheel.publish(
            self.wheels_speed/self.wheels_radius)
        self.pub_vel_right_rear_wheel.publish(
            self.wheels_speed/self.wheels_radius)
        self.pub_vel_left_front_wheel.publish(
            self.wheels_speed/self.wheels_radius)
        self.pub_vel_right_front_wheel.publish(
            self.wheels_speed/self.wheels_radius)

        self.pub_pos_left_front_steering_hinge.publish(hinge_front_left)
        self.pub_pos_right_front_steering_hinge.publish(hinge_front_right)
        self.pub_pos_left_rear_steering_hinge.publish(hinge_rear_left)
        self.pub_pos_right_rear_steering_hinge.publish(hinge_rear_right)

    def send_speed_and_distance(self, msg):

        i = msg.name.index(self.joint_name)

        if not self.is_speed_measurement_started:
            self.last_position_of_wheel = msg.position[i]
            self.is_speed_measurement_started = True
            return

        self.distance = self.distance + \
            (msg.position[i]-self.last_position_of_wheel)*self.wheels_radius
        self.wheels_speed = msg.velocity[i]*self.wheels_radius

        motion_msg = Motion()
        motion_msg.distance = self.distance
        motion_msg.speed_linear = self.wheels_speed
        motion_msg.yaw = self.imu_yaw
        motion_msg.speed_yaw = self.imu_yaw_speed

        self.pub_movement.publish(motion_msg)
        self.last_position_of_wheel = msg.position[i]

    def change_mode_callback(self, msg: UInt8):
        try:
            self.mode = self.DRIVE_MODE_NUMBERS[msg.data]
        except KeyError as e:
            rospy.logerr(
                "Wrong value of drive mode message! Current value: "+msg.data)
            return

        rospy.loginfo("Drive Mode changed on: " + self.mode)
        self.pub_drive_mode.publish(msg)

    def indicators_callback(self, msg: Indicators):
        self.pub_left_indicator.publish(msg.is_active_left)
        self.pub_right_indicator.publish(msg.is_active_right)

    def imu_callback(self, msg: Imu):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.imu_yaw = euler[2]
        self.imu_yaw_speed = msg.angular_velocity.z
    def back_dist_callback(self, msg:LaserScan):
      try:
        self.pub_back_dist.publish(msg.ranges[0])
      except:
        rospy.logerr('could not retrieve back distance from laser scan')


if __name__ == '__main__':
    bridge = SimulatorBridge()
    try:
        bridge.run()
    except ValueError as e:
        rospy.logerr("Unexpected error: "+str(e))
    except Exception as e:
        rospy.logfatal(
            "Unexpected error occurred, closing node. Exception:" + str(e))
