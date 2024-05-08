#!/usr/bin/env python3
"""
This script listens for a key/button press indicating the start signal for the robot.
When the key/button is pressed, it publishes a True signal to the 'starter' topic, and False otherwise.
"""

import rospy
from gpiozero.pins.rpigpio import RPiGPIOFactory
from gpiozero import Button
factory = RPiGPIOFactory()

from std_msgs.msg import Bool, Int8


# Logic for key press and release are reversed because of hardware architecture.
def on_key_press():
    rospy.loginfo("(STARTER TALKER) Key removed")
    pub.publish(True)
    # Sending "match-started" ID to the speaker when releasing starter
    bluetooth_choice.publish(4)
    # Attendre 90 secondes pour envoyer le son de départ du pami
    rospy.sleep(90)
    # Sending "pami" ID to the speaker
    bluetooth_choice.publish(6)
    # Envoi d'un signal d'arrêt à tous les noeuds pour arrêter le robot
    pub.publish(False)
    # Attendre 10 secondes pour atteindre la fin du temps réglementaire
    rospy.sleep(10)
    # Sending "match_end" ID to the speaker
    bluetooth_choice.publish(7)


def on_key_release():
    rospy.loginfo("(STARTER TALKER) Key inserted")


if __name__ == '__main__':
    try:
        rospy.init_node('starter_talker', anonymous=True)
        rospy.loginfo("[START] Launcher node has started. Monitoring the start key.")
        bluetooth_choice = rospy.Publisher('speaker_choice', Int8, queue_size=1)
        start_pin = rospy.get_param('/gpio/start_button_pin')
        queue_size = rospy.get_param('/queue_size')
        
        pub = rospy.Publisher('running_phase', Bool, queue_size=queue_size)
        key = Button(int(start_pin), pin_factory=factory)
        key.when_pressed = on_key_press
        key.when_released = on_key_release

        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
