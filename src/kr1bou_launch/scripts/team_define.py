#!/usr/bin/env python3
"""
Looks out for Emergency Red Button. Stops everything.
"""

import rospy
from gpiozero import Button
from std_msgs.msg import Bool, Int8


def speaker_state_callback(data: Bool):
    global speaker_state
    speaker_state = data.data
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from Speaker")


def on_button_press():
    rospy.loginfo("Team selected to blue")
    pub.publish(True)
    # Sending "Blue" ID to the speaker
    bluetooth_choice.publish(3)


def on_button_released():
    rospy.loginfo("Team selected to yellow")
    pub.publish(False)
    # Sending "Yellow" ID to the speaker
    bluetooth_choice.publish(2)


def run(data: Bool):
    global start
    start = data.data
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from RunningPhase")


if __name__ == '__main__':
    start = False
    speaker_state = False
    try:
        # Initialization
        rospy.init_node('team_define', anonymous=True)
        rospy.loginfo("[START] Team Selector node has started.")
        # Wait for the runningPhase True signal

        queue_size = rospy.get_param('/queue_size')
        pub = rospy.Publisher('Team', Bool, queue_size=queue_size)
        bluetooth_choice = rospy.Publisher('speaker_choice', Int8, queue_size=queue_size)
        rospy.Subscriber('speaker_state', Bool, speaker_state_callback)

        # GPIO setup
        button_pin = rospy.get_param('/gpio/team_button_pin')
        button = Button(int(button_pin))  # The emergency button

        frequency = rospy.get_param('/frequency')
        rate = rospy.Rate(frequency)
        # Wait for the runningPhase True signal
        rospy.Subscriber('runningPhase', Bool, run)

        rospy.sleep(1)

        rospy.loginfo("Waiting for speaker node to be ready")

        while not speaker_state:
            rate.sleep()

        rospy.loginfo("Speaker node is ready")

        if button.is_active:
            rospy.loginfo("True - Is Blue")
            pub.publish(True)
            # Sending "Blue" ID to the speaker
            bluetooth_choice.publish(3)
        else:
            rospy.loginfo("False - Is Yellow")
            pub.publish(False)
            # Sending "Yellow" ID to the speaker
            bluetooth_choice.publish(2)

        button.when_pressed = on_button_press
        button.when_released = on_button_released

        # Spin to keep the script for exiting

        rospy.Subscriber('runningPhase', Bool, run)
        rate = rospy.Rate(rospy.get_param('/frequency'))
        while not start:
            rate.sleep()

        rospy.sleep(0.1)

        if button.is_active:
            rospy.loginfo("START MATCH WITH TEAM BLUE")
            pub.publish(True)
        else:
            rospy.loginfo("START MATCH WITH TEAM YELLOW")
            pub.publish(False)

        rospy.spin()
    except rospy.ROSInterruptException as e:
        # rospy.logerr(e)
        pass
    finally:
        rospy.loginfo("[STOP] team selector node has stopped.")
