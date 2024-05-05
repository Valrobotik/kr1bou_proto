#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Int16, Bool
import time
import RPi.GPIO as GPIO

# Set function to calculate percent from angle
def angle_to_percent(angle):
    if angle > 270 or angle < 0:
        return False

    lower = 4
    upper = 12.5  # Adjust upper limit for 270-degree range
    ratio = (upper - lower) / 270  # Calculate ratio from angle to percent

    angle_as_percent = angle * ratio

    return lower + angle_as_percent

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
    try:
        # Initialization
        rospy.init_node("solar_control", anonymous=True)
        rospy.loginfo("[START] Solar Controller node has started.")
        GPIO.setmode(GPIO.BOARD)  # Use Board numeration mode
        GPIO.setwarnings(False)  # Disable warnings

        # Use pin 12 for PWM signal
        pwm_gpio = 12
        frequency = 50  # Adjust frequency as needed
        GPIO.setup(pwm_gpio, GPIO.OUT)
        pwm = GPIO.PWM(pwm_gpio, frequency)
        rospy.Subscriber("solar_angle", Int16, go_to)

        pwm.start(angle_to_percent(0))
        rospy.loginfo("Went to 0 degrees")
        rospy.sleep(10)
        pwm.stop()
        pwm.start(angle_to_percent(150))
        rospy.loginfo("Went to 150 degrees")
        rospy.sleep(3)
        pwm.stop()
        rospy.spin()
    finally:
        if pwm is not None:
            pwm.stop()
        GPIO.cleanup()
        rospy.loginfo("[STOP] Solar Controller node has stopped.")
