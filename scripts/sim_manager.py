#!/usr/bin/env python
"""Module for keyboard ackerman steering."""
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from pynput import keyboard


class SimManager(object):

    """Class for keyboard ackerman steering manager. """
    class SteeringMode(object):
        """Class for managing steering modes"""

        def __init__(self):
            self.modes = ['MANUAL', 'SEMI_AUTOMATIC', 'FULL_AUTOMATIC']
            self.active_mode = 0

        def change_mode(self):
            """Switch to next mode"""
            if(self.active_mode == 2):
                self.active_mode = 0
            else:
                self.active_mode += 1
            rospy.loginfo(str(self.get_active_mode()) + ' mode')

        def get_active_mode(self):
            """Get active mode as a string"""
            return self.modes[self.active_mode]

    def __init__(self, max_steering_angle=0.55, max_speed=1):
        self.max_steering_angle = max_steering_angle
        self.max_speed = max_speed
        self.steering_mode = self.SteeringMode()
        self.command = AckermannDriveStamped()
        self.incoming_drive_command = AckermannDriveStamped()
        self.on_press_response = {
            keyboard.Key.ctrl_r: self.steering_mode.change_mode,
            keyboard.Key.left: self.turn_left,
            keyboard.Key.right: self.turn_right,
            keyboard.Key.up: self.go_forward,
            keyboard.Key.down: self.go_backwards
            }
        self.on_release_response = {
            keyboard.Key.left: self.reset_steering_angle,
            keyboard.Key.right: self.reset_steering_angle,
            keyboard.Key.up: self.reset_speed,
            keyboard.Key.down: self.reset_speed
        }

    def drive_callback(self, msg):
        self.incoming_drive_command = msg

    def reset_steering_angle(self):
        self.command.drive.steering_angle = 0

    def reset_speed(self):
        self.command.drive.speed = 0

    def go_forward(self):
        if(self.command.drive.speed < self.max_speed):
            self.command.drive.speed += 0.1

    def turn_left(self):
        if(self.command.drive.steering_angle > -self.max_steering_angle):
            self.command.drive.steering_angle -= 0.1

    def turn_right(self):
        if(self.command.drive.steering_angle < self.max_steering_angle):
            self.command.drive.steering_angle += 0.1

    def go_backwards(self):
        if(self.command.drive.speed > -self.max_speed):
            self.command.drive.speed -= 0.1

    def on_press(self, key_pressed):
        try:
            self.on_press_response[key_pressed]()
        except KeyError:
            pass

    def on_release(self, key_released):
        try:
            self.on_release_response[key_released]()
        except KeyError:
            pass

    def publish(self, publisher):
        msg = None
        if(self.steering_mode.get_active_mode() == 'FULL_AUTOMATIC'):
            msg = self.incoming_drive_command
        elif(self.steering_mode.get_active_mode() == 'SEMI_AUTOMATIC'):
            msg = self.command
            msg.drive.steering_angle = self.incoming_drive_command.drive.steering_angle
        else:
            msg = self.command
        publisher.publish(msg)


if __name__ == '__main__':
    rospy.init_node('sim_manager', anonymous=True)
    MANAGER = SimManager()
    PUB = rospy.Publisher('/sim_drive', AckermannDriveStamped, queue_size=1)
    rospy.Subscriber('/drive', AckermannDriveStamped, MANAGER.drive_callback)

    LISTENER = keyboard.Listener(on_press=MANAGER.on_press, on_release=MANAGER.on_release)
    LISTENER.start()
    RATE = rospy.Rate(2)
    while not rospy.is_shutdown():
        MANAGER.publish(PUB)
        RATE.sleep()

    LISTENER.stop()
    rospy.signal_shutdown("manually closed")
