#!/usr/bin/env python3
"""

"""

import rospy
import serial
from std_msgs.msg import Bool, Int8


def run(data: Bool):
    global start, ser
    start = data.data
    if start:
        ser.write(b'RDY\r')
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from RunningPhase")


def update_team(data: Bool):
    global team, ser
    if data.data:
        ser.write(b'B\r')
        team = TEAM_BLUE
    else:
        ser.write(b'Y\r')
        team = TEAM_YELLOW
    rospy.sleep(3)
    pts = Int8(18)
    change_pts(pts)

def change_pts(data : Int8):
    cmd = "S"+str(data.data)+"\r"
    ser.write(cmd.encode())


if __name__ == '__main__':
    start = False
    team = -1
    TEAM_BLUE = 1
    TEAM_YELLOW = 0
    try:
        # Initialization
        rospy.init_node('ui_controller', anonymous=True)

        ser = serial.Serial(rospy.get_param("/arduino/arduino_serial_ports/UI_Controller"), rospy.get_param("/arduino/baudrate"))
        rospy.Subscriber("team", Bool, update_team)

        rospy.Subscriber('running_phase', Bool, run)
        rospy.Subscriber('points', Int8, change_pts)
        rospy.sleep(6)
        ser.write(b'INI\r')
        
        rospy.spin()
    except Exception as e:
        rospy.logerr("Error on UI Controller")
        rospy.logerr(e)
