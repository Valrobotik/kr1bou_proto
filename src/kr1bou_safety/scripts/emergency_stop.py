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

start = False
def run(data:Bool):
    global start
    start = data.data
    rospy.loginfo(f"Received {start} from runningPhase")

if __name__ == '__main__':
    try:
        # Initialization
        rospy.init_node('emergency_stop', anonymous=True)
        rospy.loginfo("[START] Emergency Stop node has started.")
        # # Wait for the runningPhase True signal
        # frequency = rospy.get_param('/frequency')
        # rate = rospy.Rate(frequency)
        # # Wait for the runningPhase True signal
        # rospy.Subscriber('runningPhase', Bool, run)
        # while not start:
        #     rate.sleep()
        # # Load configuration
        # button_pin = rospy.get_param('/gpio/emergency_button_pin')

        # # ROS node initialization
        # rospy.init_node('emergency_stop', anonymous=True)
        # queue_size = rospy.get_param('/queue_size')
        # pub = rospy.Publisher('emergency', Bool, queue_size=queue_size)

        # # GPIO setup
        # button = Button(int(button_pin))  # The emergency button
        # button.when_pressed = on_button_press

        # # Spin to keep the script for exiting
        # rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)

