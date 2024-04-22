#!/usr/bin/env python3
"""
Controls the movement of the robot using inputs from sensors and navigation data.
"""
from math import atan2, sqrt, cos, sin, atan, pi

import rospy
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float64, Bool


READY_LINEAR = 0
READY = 1
IN_PROGESS = 2

KLigne = -19.0
KAngle = 0.1

ANGLE_PRECISION = 0.05
DISTANCE_PRECISION = 0.05
KP_R = -2.0

GOTO_DELTA = -0.02
POSITION_SHIFT = 0.0

GOTO_BASE_DISTANCE_THRESHOLD = 0.04

ROTATE_TIME = 1.0

WHEEL_FORWARD_SPEED = 0.25
WHEEL_BACKWARD_SPEED = 0.25
WHEEL_TURN_SPEED_FORWARD = 0.30
WHEEL_TURN_SPEED_BACKWARD = 0.30

WHEEL_HIGHSPEED_FACTOR = 2.0


class Kr1bou():
    def __init__(self):
        self.etat = READY

        self.x = 0
        self.y = 0
        self.theta = 0
        
        self.objectif_x = 0.2
        self.objectif_y = 0
        self.objectif_theta = 0

        self.vitesse_gauche = 0
        self.vitesse_droite = 0

        self.last_left_speed = 0
        self.last_right_speed = 0

        self.freq = rospy.get_param('/frequency')

        self.publisher_speed = rospy.Publisher('motor_speed', Vector3, queue_size=1)
        rospy.Subscriber('odometry', Pose2D, self.update_pose)
        rospy.Subscriber('next_objectif', Pose2D, self.set_objectif)
        rospy.Subscriber('max_speed', Float64, self.set_max_speed)
        rospy.Subscriber('stop', Bool, self.stop)

    def publish_speed(self):
        if self.etat == IN_PROGESS :
            self.update_speed()
        elif self.etat == READY_LINEAR and self.objectif_theta != -1:
            self.update_rotation_speed()
        elif self.etat == READY_LINEAR :
            self.etat = READY
            self.vitesse_gauche = 0
            self.vitesse_droite = 0
        else:
            self.vitesse_gauche = 0
            self.vitesse_droite = 0
        data = Vector3()
        data.x = self.vitesse_gauche
        data.y = self.vitesse_droite
        self.publisher_speed.publish(data)


    def update_rotation_speed(self):
        angle_diff = self.objectif_theta - self.theta
        w = angle_diff * KP_R
        if abs(angle_diff) < ANGLE_PRECISION:
            self.etat = READY
            w = 0
        self.vitesse_gauche = w
        self.vitesse_droite = -w

    def update_pose(self, data:Pose2D):
        self.x = data.x
        self.y = data.y
        self.theta = data.theta

    def update_speed(self, allowed_backward = False):
        x2 = self.objectif_x
        y2 = self.objectif_y

        destination_angle = atan2(y2 - self.y, x2 - self.x)

        pos = [self.x, self.y]
        pos[0] -= cos(self.theta) * POSITION_SHIFT
        pos[1] -= sin(self.theta) * POSITION_SHIFT

        p1 = [x2, y2]
        p2 = [x2 + cos(destination_angle), y2 + sin(destination_angle)]

        dist_to_line = abs((p2[0] - p1[0]) * (p1[1] - pos[1]) - (p1[0] - pos[0]) * (p2[1] - p1[1]) / sqrt((p2[0] - p1[0]) * (p2[0] - p1[0]) + (p2[1] - p1[1]) * (p2[1] - p1[1])))
                           
        angle_line_to_robot = atan2(pos[1] - p1[1], pos[0] - p1[0])
        
        diff_angle_line_to_robot = destination_angle - angle_line_to_robot
        is_on_right_side = diff_angle_line_to_robot > 0.0
        target_angle = destination_angle + atan(dist_to_line * 3.5) * (-1 if is_on_right_side else 1)

        dx_base = x2 - pos[0]
        dy_base = y2 - pos[1]

        dist_to_base = sqrt(dx_base * dx_base + dy_base * dy_base)

        m_goto_base_reached = False
        if dist_to_base < GOTO_BASE_DISTANCE_THRESHOLD:
            m_goto_base_reached = True
            self.etat = READY_LINEAR
        
        angle_diff = target_angle - self.theta
        backward = abs(angle_diff) > pi / 2

        backward = backward and allowed_backward
        if backward:
            angle_diff = target_angle + pi - self.theta

        speed_limit = WHEEL_BACKWARD_SPEED if backward else WHEEL_FORWARD_SPEED

        forward_speed = speed_limit * (1 - abs(angle_diff) / (pi / 2))

        if m_goto_base_reached:
            forward_speed *= 0.1
        if backward:
            forward_speed = -forward_speed
        
        turn_speed = abs(angle_diff) / (pi / 2)
        turn_speed *= (WHEEL_TURN_SPEED_BACKWARD if backward else WHEEL_TURN_SPEED_FORWARD)
        turn_speed *= 1 if angle_diff > 0 else -1

        reduction_factor = 1
        left_speed = reduction_factor * (forward_speed + turn_speed)
        right_speed = reduction_factor * (forward_speed - turn_speed)

        if self.etat == READY_LINEAR:
            left_speed = 0
            right_speed = 0

        self.vitesse_gauche = left_speed
        self.vitesse_droite = right_speed

        self.last_left_speed = left_speed
        self.last_right_speed = right_speed

    def set_objectif(self, data:Pose2D):
        self.etat = IN_PROGESS
        self.objectif_x = data.x
        self.objectif_y = data.y
        self.objectif_theta = data.theta

    def set_max_speed(self, data:Float64):
        global WHEEL_FORWARD_SPEED, WHEEL_BACKWARD_SPEED
        WHEEL_FORWARD_SPEED = data.data
        WHEEL_BACKWARD_SPEED = data.data

    def stop(self, data:Bool):
        if data.data:
            self.etat = READY
            self.vitesse_gauche = 0
            self.vitesse_droite = 0
        else:
            self.etat = IN_PROGESS

    def __del__(self):
        self.serial_port.close()


if __name__=="__main__":
    try:
        # Initialization
        rospy.init_node('controller', anonymous=True)
        # Wait for the runningPhase True signal
        start = rospy.wait_for_message('runningPhase', Bool)
        while not start.data:
            start = rospy.wait_for_message('runningPhase', Bool)
        robot = Kr1bou()
        rate = rospy.Rate(robot.freq)
        while not rospy.is_shutdown():  
            robot.publish_speed()
            rate.sleep()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
    finally:
        del robot
