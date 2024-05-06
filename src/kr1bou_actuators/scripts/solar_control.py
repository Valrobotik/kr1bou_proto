#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Int16
from piServo.Servo import servo

PIN = 13


# Set function to calculate percent from angle
def convert_angle(angle: int):
    if 0 > angle > 180:
        return 0
    return int(120 - (angle * 180 / 270))


def rotate_to(data: Int16):
    global solar_arm
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from solar_angle")
    angle = data.data
    solar_arm.write(convert_angle(angle))
    rospy.sleep(0.5)
    rospy.loginfo(f"{rospy.get_name()} sent: {convert_angle(angle)} to Servo")


if __name__ == "__main__":
    # Initialization
    rospy.init_node("solar_control", anonymous=True)
    rospy.loginfo("[START] Solar Controller node has started.")

    # Subscribe to solar_angle
    rospy.Subscriber("solar_angle", Int16, rotate_to)

    # Initialize servo
    solar_arm = servo(13)
    solar_arm.write(convert_angle(0))
    rospy.sleep(0.5)
    solar_arm.write(convert_angle(90))

    rospy.spin()
    rospy.loginfo("[STOP] Solar Controller node has stopped.")
