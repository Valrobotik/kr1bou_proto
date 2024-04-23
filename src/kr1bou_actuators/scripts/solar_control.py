#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Int16, Bool

import RPi.GPIO as GPIO


# Set function to calculate percent from angle
def angle_to_percent(angle):
    if angle > 180 or angle < 0:
        return False

    lower = 4
    upper = 12.5
    ratio = (upper - lower) / 180  # Calcul ratio from angle to percent

    angle_as_percent = angle * ratio

    return lower + angle_as_percent


def go_to(data: Int16):
    global pwm
    pwm.ChangeDutyCycle(angle_to_percent(data.data))
    rospy.loginfo(data.data)


def run(data: Bool):
    global start
    start = data.data
    rospy.loginfo(f"Received {start} from runningPhase")


if __name__ == "__main__":
    start = False
    pwm = None
    try:
        # Initialization
        rospy.init_node("solar_control", anonymous=True)
        rospy.loginfo("[START] Wheel Controller node has started.")
        GPIO.setmode(GPIO.BOARD)  # Use Board numeration mode
        GPIO.setwarnings(False)  # Disable warnings

        # Use pin 12 for PWM signal
        pwm_gpio = 12
        frequency = 60
        GPIO.setup(pwm_gpio, GPIO.OUT)
        pwm = GPIO.PWM(pwm_gpio, frequency)
        rospy.Subscriber("solar_angle", Int16, go_to)

        pwm.start(angle_to_percent(0))

        rospy.spin()

    except rospy.ROSInterruptException as e:
        pass
    finally:
        if pwm is not None:
            pwm.stop()
        GPIO.cleanup()
        rospy.loginfo("Wheel Controller node has stopped.")
