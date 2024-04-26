#!/usr/bin/env python3
"""
Looks out for Emergency Red Button. Stops everything.
"""

import rospy
from gpiozero import Button
from std_msgs.msg import Bool


def on_button_press():
    rospy.loginfo("Team selected to blue")
    pub.publish(True)


def run(data: Bool):
    global start
    start = data.data
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from RunningPhase")


if __name__ == '__main__':
    start = False
    try:
        # Initialization
        rospy.init_node('team_define', anonymous=True)
        rospy.loginfo("[START] Emergency Stop node has started.")
        # Wait for the runningPhase True signal

        queue_size = rospy.get_param('/queue_size')
        pub = rospy.Publisher('Team', Bool, queue_size=queue_size)

                # GPIO setup
        button_pin = rospy.get_param('/gpio/team_button_pin')
        button = Button(int(button_pin))  # The emergency button

        frequency = rospy.get_param('/frequency')
        rate = rospy.Rate(frequency)
        # Wait for the runningPhase True signal
        rospy.Subscriber('runningPhase', Bool, run)
        while not start:
            rate.sleep()
        # Load configuration

        if button.is_pressed:
            rospy.loginfo("TEAM BLUE")
            pub.publish(True)
        else : 
            rospy.loginfo("TEAM YELLOW")
            pub.publish(False)

        # Spin to keep the script for exiting
        rospy.spin()
    except rospy.ROSInterruptException as e:
        # rospy.logerr(e)
        pass
    finally:
        rospy.loginfo("[STOP] Emergency Stop node has stopped.")
