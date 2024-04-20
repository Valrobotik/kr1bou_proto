#!/usr/bin/env python3
"""
Looks out for bumpers signals on specific GPIO pins. Handles events such as the robot reaching a wall.
When two front or two back buttons are held, it publishes a True signal on the 'bumper' topic.
"""

import rospy
from gpiozero import Button
from std_msgs.msg import Bool

def setup_buttons(pins):
    return [Button(pin) for pin in pins]


def check_bumpers(front_buttons, back_buttons):
    # Check if both front or both back buttons are pressed
    front_pressed = all(button.is_pressed for button in front_buttons)
    back_pressed = all(button.is_pressed for button in back_buttons)
    return front_pressed or back_pressed


def publish_bumper_event(front_pins, back_pins, frequency):
    front_buttons = setup_buttons(front_pins)
    back_buttons = setup_buttons(back_pins)
    
    pub = rospy.Publisher('bumper', Bool, queue_size=10)
    rospy.init_node('bumper_watch', anonymous=True)
    rate = rospy.Rate(frequency)

    while not rospy.is_shutdown():
        bumper_pressed = check_bumpers(front_buttons, back_buttons)
        pub.publish(bumper_pressed)
        rate.sleep()


if __name__ == '__main__':
    try:
        front_pins = rospy.get_param('/gpio/front_bumper_pins')
        back_pins = rospy.get_param('/gpio/back_bumper_pins')
        frequency = rospy.get_param('/frequency', 10)  # Default to 10 Hz if not set
        publish_bumper_event(front_pins, back_pins, frequency)
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
