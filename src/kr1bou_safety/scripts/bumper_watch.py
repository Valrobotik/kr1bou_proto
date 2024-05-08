#!/usr/bin/env python3
"""
Looks out for bumpers signals on specific GPIO pins. Handles events such as the robot reaching a wall.
When two front or two back buttons are held, it publishes on the 'bumper' topic.
"""
import rospy
from std_msgs.msg import Byte, Bool
from gpiozero.pins.rpigpio import RPiGPIOFactory
from gpiozero import Button
from typing import List

factory = RPiGPIOFactory()

def on_bumper_press():
    global buttons_list
    state = check_bumpers(buttons_list)
    pub.publish(state)


def on_bumper_release():
    state = check_bumpers(buttons_list)
    pub.publish(state)


def setup_buttons(pins):
    global buttons_list
    buttons_list = [Button(int(pin), pin_factory=factory) for pin in pins]
    #rospy.loginfo(f"(BUMPER WATCH) {buttons_list}")
    buttons_list[0].when_activated = on_bumper_press
    buttons_list[0].when_activated = on_bumper_release
    old_state = check_bumpers(buttons_list)
    custom_rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        state = check_bumpers(buttons_list)
        if state.data != old_state.data:
            pub.publish(state)
            old_state = state
        custom_rate.sleep()


def check_bumpers(buttons: List[Button]):
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
        # Wait for the running_phase True signal
        frequency = rospy.get_param('/frequency')
        rate = rospy.Rate(frequency)
        # Wait for the running_phase True signal
        rospy.Subscriber('running_phase', Bool, run)
        bumper_pins = rospy.get_param('/gpio/bumper_pins')
        frequency = rospy.get_param('/frequency')
        queue_size = rospy.get_param('/queue_size')
        pub = rospy.Publisher('bumper', Byte, queue_size=queue_size)
        setup_buttons(bumper_pins)
    finally:
        rospy.loginfo("[STOP] Bumper Watch node has stopped.")
