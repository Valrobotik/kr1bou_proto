#!/usr/bin/env python3
#-- coding: utf-8 --

import rospy
from std_msgs.msg import Int16

import RPi.GPIO as GPIO
import time


#Set function to calculate percent from angle
def angle_to_percent (angle) :
    if angle > 180 or angle < 0 :
        return False

    start = 4
    end = 12.5
    ratio = (end - start)/180 #Calcul ratio from angle to percent

    angle_as_percent = angle * ratio

    return start + angle_as_percent

def go_to(data:Int16):
    global pwm
    pwm.ChangeDutyCycle(angle_to_percent(data.data))

if __name__ == "__main__":
    rospy.init_node("solar_control", anonymous=True)
    GPIO.setmode(GPIO.BOARD) #Use Board numerotation mode
    GPIO.setwarnings(False) #Disable warnings

    #Use pin 12 for PWM signal
    pwm_gpio = 12
    frequence = 50
    GPIO.setup(pwm_gpio, GPIO.OUT)
    pwm = GPIO.PWM(pwm_gpio, frequence)
    rospy.Subscriber("solar_angle", Int16, go_to)

    go_to(0)

    rospy.spin()
    pwm.stop()
    GPIO.cleanup()

