#!/usr/bin/env python3
"""
Looks out for bumpers signals on specific GPIO pins. Handles events such as the robot reaching a wall.
When two front or two back buttons are held, it publishes on the 'bumper' topic.
"""
import rospy
from std_msgs.msg import Byte, Bool
from gpiozero import Button
from typing import List


def on_bumper_press():
    global buttons_list
    rospy.loginfo("(Bumper) Pressed")
    state = check_bumpers(buttons_list)
    pub.publish(state)


def on_bumper_release():
    rospy.loginfo("(Bumper) Release")
    state = check_bumpers(buttons_list)
    pub.publish(state)


def setup_buttons(pins):
    global buttons_list
    buttons_list = [Button(int(pin)) for pin in pins]
    rospy.loginfo(f"(BUMPER WATCH) {buttons_list}")
    buttons_list[0].when_pressed = on_bumper_press
    buttons_list[0].when_released = on_bumper_release
    old_state = check_bumpers(buttons_list)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        state = check_bumpers(buttons_list)
        if state.data != old_state.data:
            pub.publish(state)
            old_state = state
        rate.sleep()


def check_bumpers(buttons: List[Button]):
    # Check if both front or both back buttons are pressed
    state = Byte()
    state.data = 0
    for i, button in enumerate(buttons):
        if button.is_active:
            state.data += 2**i
    state.data = 15-state.data
    return state


def run(data: Bool):
    global start
    start = data.data
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from RunningPhase")


if __name__ == '__main__':
    start = False
    buttons_list = []
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
        setup_buttons(bumper_pins)
    finally:
        rospy.loginfo("[STOP] Bumper Watch node has stopped.")
