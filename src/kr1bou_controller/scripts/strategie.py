#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float64, Bool, Int16, Float32MultiArray, Byte

READY_LINEAR = 0
READY = 1
IN_PROGESS = 2

class strategie():
    def __init__(self) -> None:
        rospy.init_node("strategie")
        rospy.loginfo("[START] strategie node has started.")

        self.position = Pose2D()
        rospy.Subscriber("odometry", Pose2D, self.update_position)

        self.US_data = Float32MultiArray()
        rospy.Subscriber('ultrasound_sensor_data', Float32MultiArray, self.update_us_data)

        self.bumper_1 = False
        self.bumper_2 = False
        self.bumper_3 = False
        self.bumper_4 = False
        rospy.Subscriber('bumper', Byte, self.update_bumpers)

        self.etat_robot = READY
        rospy.Subscriber('state', Int16, self.update_state)

        self.solar_pub = rospy.Publisher('solar_angle', Int16, queue_size=1)
        self.pos_ordre_pub = rospy.Publisher('next_objectif', Pose2D, queue_size=1)
        self.speed_ctrl = rospy.Publisher('max_speed', Float64, queue_size=1)

    def update_bumpers(self, data):
        """met à jours l'etat des 4 bumper (2 avans et 2 arierre)"""
        if data>=8:
            self.bumper_1 = True
            data-=8
        else : self.bumper_1 = False
        if data >=4:
            self.bumper_2 = True
            data-=4
        else : self.bumper_2 = False
        if data >=2:
            self.bumper_3 = True
            data-=2
        else : self.bumper_3 = False
        if data >=1:
            self.bumper_4 = True
            data-=1
        else : self.bumper_4 = False

    def update_position(self, data):
        self.position = data

    def update_us_data(self, data):
        self.US_data = data

    def update_state(self, data):
        self.etat_robot = data

    def go_to(self, x=-1, y=-1, alpha = -1, speed = 0.25):
        obj = Pose2D()
        obj.x = x
        obj.y = y
        obj.theta = alpha
        self.pos_ordre_pub.publish(obj)

    
    def turn_servo(self, alpha):
        a = Int16()
        a.data = alpha
        self.solar_pub.publish(a)

    def wait_until_ready(self):
        rospy.sleep(0.1)
        while self.etat_robot != READY : pass

    def run(self):
        while not rospy.is_shutdown():
            self.go_to(1, 0)
            self.wait_until_ready()
            self.turn_servo(90)

            


if __name__ == "__main__":
    strat = strategie()
    strat.run()

