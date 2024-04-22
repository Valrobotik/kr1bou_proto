#!/usr/bin/env python3
"""
This script listens for a key/button press indicating the start signal for the robot.
When the key/button is pressed, it publishes a True signal to the 'starter' topic, and False otherwise.
"""

import rospy
from gpiozero import Button
from std_msgs.msg import Bool


def on_key_press():
    rospy.loginfo("Key inserted: True")
    pub.publish(True)


def on_key_release():
    rospy.loginfo("Key removed: False")
    pub.publish(False)


if __name__ == '__main__':
    try:
        rospy.init_node('starter_talker', anonymous=True)
        start_pin = rospy.get_param('/gpio/start_button_pin')
        queue_size = rospy.get_param('/queue_size')
        
        pub = rospy.Publisher('configPhase', Bool, queue_size=queue_size)
        key = Button(start_pin)
        key.when_pressed = on_key_press
        key.when_released = on_key_release

        rospy.loginfo("Starter node initialized, monitoring the start key.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
