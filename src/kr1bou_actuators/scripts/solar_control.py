#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Int16, Bool
from piservo import Servo


# Set function to calculate percent from angle
def convert_angle(angle: int):
    """Maps 180 to 270 degrees"""

    return int(angle * 180 / 270)


def go_to(data: Int16):
    global pwm
    pwm.start(angle_to_percent(data.data))
    rospy.loginfo(f"(SOLAR_CONTROL) {data.data}")
    rospy.sleep(3)
    pwm.stop()

def run(data: Bool):
    global start
    start = data.data
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from RunningPhase")


if __name__ == "__main__":
    start = False
    pwm = None
    time_run = 0
    # Initialization
    rospy.init_node("solar_control", anonymous=True)
    rospy.loginfo("[START] Solar Controller node has started.")

    # Use pin 12 for PWM signal
    pwm_gpio = 12
    frequency = 50
    rospy.Subscriber("solar_angle", Int16, go_to)

    servo = Servo(13)
    servo.write(0)
    rospy.sleep(3)
    servo.write(convert_angle(45))
    rospy.sleep(3)
    servo.write(convert_angle(90))
    rospy.sleep(3)
    servo.write(convert_angle(135))
    rospy.sleep(3)
    servo.write(convert_angle(180))
    rospy.sleep(3)

    rospy.spin()
    rospy.loginfo("[STOP] Solar Controller node has stopped.")