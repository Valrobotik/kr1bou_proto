#!/usr/bin/env python3
"""
Looks out for bumpers signals on specific GPIO pins. Handles events such as the robot reaching a wall.
When two front or two back buttons are held, it publishes on the 'bumper' topic.
"""
import rospy
from std_msgs.msg import Byte, Bool
from gpiozero import Button
from typing import List


def setup_buttons(pins):
    rospy.loginfo(f"(BUMPER WATCH) {pins}")
    return [Button(int(pin)) for pin in pins]


def check_bumpers(buttons: List[Button]):
    # Check if both front or both back buttons are pressed
    state = Byte()
    state.data = 0
    for i, button in enumerate(buttons):
        if button.is_active:
            state.data += 2**i
    return state


def publish_bumper_event(bumpers):
    global pub
    buttons = setup_buttons(bumpers)

    while not rospy.is_shutdown():
        bumpers_pressed = check_bumpers(buttons)
        pub.publish(bumpers_pressed)
        # rospy.loginfo(f"bumper data : {bumpers_pressed.data}")
        rate.sleep()


def run(data: Bool):
    global start
    start = data.data
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from RunningPhase")


if __name__ == '__main__':
    start = False
    try:
        # Initialization
        rospy.init_node('bumper_watch', anonymous=True)
        rospy.loginfo("[START] Bumper Watch node has started.")
        # Wait for the runningPhase True signal
        frequency = rospy.get_param('/frequency')
        rate = rospy.Rate(frequency)
        # Wait for the runningPhase True signal
        rospy.Subscriber('runningPhase', Bool, run)
        bumper_pins = rospy.get_param('/gpio/bumper_pins')
        frequency = rospy.get_param('/frequency')
        queue_size = rospy.get_param('/queue_size')
        pub = rospy.Publisher('bumper', Byte, queue_size=queue_size)
        while not start:
            rate.sleep()
        publish_bumper_event(bumper_pins)
    finally:
        rospy.loginfo("[STOP] Bumper Watch node has stopped.")
