#!/usr/bin/env python3
"""

"""

import rospy
import serial
from std_msgs.msg import Bool

TEAM_BLUE = 1
TEAM_YELLOW = 0

start = 0
def run(data: Bool):
    global start, ser
    start = data.data
    if start : ser.write(b'RDY\r')
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from RunningPhase")

team = -1
def update_team(data : Bool):
    global team, ser
    if data.data : 
        ser.write(b'B\r')
        team = TEAM_BLUE
    else :
        ser.write(b'Y\r')
        team = TEAM_YELLOW

    
if __name__ == '__main__':
    start = False
    try:
        # Initialization
        rospy.init_node('UI_controler', anonymous=True)

        ser = serial.Serial("/dev/ttyUSB0", 115200)
        rospy.Subscriber("Team", Bool, update_team)

        rospy.Subscriber('runningPhase', Bool, run)
        rospy.sleep(1)
        ser.write(b'INI\r')
        rospy.spin()
    except Exception as e:
        rospy.logerr("Error on UI Controler")
        rospy.logerr(e)
