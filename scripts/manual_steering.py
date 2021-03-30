#!/usr/bin/env python3
"""Module for keyboard ackermann steering."""
import rospy
from custom_msgs.msg import DriveCommand
from std_msgs.msg import Int8
from pynput import keyboard


class ManualSteering(object):
    class DriveMode(object):
        MANUAL = 0
        AUTOMATIC = 1
        SEMI_AUTOMATIC = 2

    def __init__(self):
        rospy.init_node("manual_steering", anonymous=False)

        # Inner variables and objects
        self.current_drive_mode = self.DriveMode.AUTOMATIC
        self.current_speed = 1.2
        self.speed_change_step = 0.05
        self.base_steering_angle = 0.5
        self.drive_message = DriveCommand()
        self.drive_message.steering_angle_rear = 0

        self.hotkey_switch_mode = keyboard.HotKey(
            keyboard.HotKey.parse("<ctrl>+n"), self.switch_mode)
        self.hotkey_increase_speed = keyboard.HotKey(
            keyboard.HotKey.parse("a+w"),
            lambda: self.change_speed(self.speed_change_step))
        self.hotkey_decrease_speed = keyboard.HotKey(
            keyboard.HotKey.parse("a+s"),
            lambda: self.change_speed(-self.speed_change_step))
        self.keyboard_listener = keyboard.Listener(on_press=self.on_press,
                                                   on_release=self.on_release)
        self.keyboard_listener.start()

        # Init Publishers
        self.pub_drive = rospy.Publisher("/drive/manual",
                                         DriveCommand,
                                         queue_size=1)
        self.pub_switch_state = rospy.Publisher("/simulation/switch_state",
                                                Int8,
                                                queue_size=1)

        # Others
        rospy.loginfo("manual_steering initialized")
        print("""Usage manual:
          Control speed and direction: Arrows
          Switch mode: Ctrl+N
          Changing speed: A+W/A+S
        """)

    def run(self):
        rospy.spin()

    def on_press(self, key: keyboard.Key):
        # TODO add changing drive modes and changing speed
        self.hotkey_switch_mode.press(key)
        self.hotkey_increase_speed.press(key)
        self.hotkey_decrease_speed.press(key)

        if key == keyboard.Key.up:
            self.drive_message.speed = self.current_speed
        elif key == keyboard.Key.down:
            self.drive_message.speed = -self.current_speed
        elif key == keyboard.Key.left:
            self.drive_message.steering_angle_front = self.base_steering_angle
        elif key == keyboard.Key.right:
            self.drive_message.steering_angle_front = -self.base_steering_angle

        self.pub_drive.publish(self.drive_message)

    def on_release(self, key: keyboard.Key):
        self.hotkey_switch_mode.release(key)
        self.hotkey_increase_speed.release(key)
        self.hotkey_decrease_speed.release(key)

        if key == keyboard.Key.up or key == keyboard.Key.down:
            self.drive_message.speed = 0
        elif key == keyboard.Key.left or key == keyboard.Key.right:
            self.drive_message.steering_angle_front = 0

        self.pub_drive.publish(self.drive_message)

    def switch_mode(self):
        self.current_drive_mode = (self.current_drive_mode + 1) % 3

        self.pub_switch_state.publish(Int8(self.current_drive_mode))

    def change_speed(self, change):
        self.current_speed += change


if __name__ == '__main__':
    try:
        base = ManualSteering()
        base.run()
    except ValueError as e:
        rospy.logerr("Unexpected error: " + str(e))
    except Exception as e:
        rospy.logfatal("Unexpected error occurred, closing node. Exception:" +
                       str(e))
