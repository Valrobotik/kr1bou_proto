#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Int16, Bool
from piservo import Servo

PIN = 13


# Set function to calculate percent from angle
def convert_angle(angle: int):
    if 0 > angle > 180:
        return 0
    return int(120 - (angle * 180 / 270))


def rotate_to(data: Int16):
    global servo
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from solar_angle")
    angle = data.data
    servo.write(convert_angle(angle))
    rospy.sleep(0.5)
    rospy.loginfo(f"{rospy.get_name()} sent: {convert_angle(angle)} to Servo")


if __name__ == "__main__":
    # Initialization
    rospy.init_node("solar_control", anonymous=True)
    rospy.loginfo("[START] Solar Controller node has started.")

    # Subscribe to solar_angle
    rospy.Subscriber("solar_angle", Int16, rotate_to)

    # Initialize servo
    servo = Servo(13)
    servo.write(convert_angle(0))

    rospy.spin()
    rospy.loginfo("[STOP] Solar Controller node has stopped.")