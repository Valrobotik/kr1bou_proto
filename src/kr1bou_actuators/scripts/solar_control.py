#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Int16, Bool
import time
import RPi.GPIO as GPIO


# Set function to calculate percent from angle
def angle_to_percent(angle):
    if angle > 180 or angle < 0:
        return 0
    
    return angle * 100 / 270


def rotate_to(data: Int16):
    global pwm
    pwm.start(angle_to_percent(data.data))
    rospy.loginfo(f"(SOLAR_CONTROL) Rotating to {data.data}")
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
    try:
        # Initialization
        rospy.init_node("solar_control", anonymous=True)
        rospy.loginfo("[START] Solar Controller node has started.")
        GPIO.setmode(GPIO.BOARD)  # Use Board numeration mode
        GPIO.setwarnings(False)  # Disable warnings

        # Use pin 12 for PWM signal
        pwm_gpio = 12
        frequency = 50
        GPIO.setup(pwm_gpio, GPIO.OUT)
        pwm = GPIO.PWM(pwm_gpio, frequency)
        rospy.Subscriber("solar_angle", Int16, rotate_to)

        rotate_to(Int16(0))
        rotate_to(Int16(90))
        rospy.spin()
    finally:
        if pwm is not None:
            pwm.stop()
        GPIO.cleanup()
        rospy.loginfo("[STOP] Solar Controller node has stopped.")