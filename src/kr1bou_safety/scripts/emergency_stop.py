#!/usr/bin/env python3
"""
Looks out for Emergency Red Button. Stops everything.
"""

import rospy
from gpiozero import Button
from std_msgs.msg import Bool


def on_button_press():
    rospy.loginfo("Emergency button pressed!")
    pub.publish(True)


if __name__ == '__main__':
    try:
        # Load configuration
        button_pin = rospy.get_param('/gpio/emergency_button_pin')

        # ROS node initialization
        rospy.init_node('emergency_stop', anonymous=True)
        pub = rospy.Publisher('emergency', Bool, queue_size=10)

        # GPIO setup
        button = Button(button_pin)  # The emergency button
        button.when_pressed = on_button_press

        # Spin to keep the script for exiting
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)

