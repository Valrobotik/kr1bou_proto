#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Int16, Bool
import time
import RPi.GPIO as GPIO
from gpiozero import AngularServo


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
    time_run = 0
    # Initialization
    rospy.init_node("solar_control", anonymous=True)
    rospy.loginfo("[START] Solar Controller node has started.")

    pwm_gpio = 12
    # frequency = 50
    # GPIO.setup(pwm_gpio, GPIO.OUT)
    # pwm = GPIO.PWM(pwm_gpio, frequency)

    servo = AngularServo(pwm_gpio, initial_angle=135, min_angle=0, max_angle=270, min_pulse_width=1/1000, max_pulse_width=2.5/1000)
    rospy.loginfo(f"Servo initialized on GPIO {pwm_gpio}")

    #rospy.Subscriber("solar_angle", Int16, rotate_to)

    # rotate_to(Int16(0))
    # rotate_to(Int16(180))
    rospy.spin()

    rospy.loginfo("[STOP] Solar Controller node has stopped.")