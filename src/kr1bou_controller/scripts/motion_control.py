#!/usr/bin/env python3
"""
Controls the movement of the robot using inputs from sensors and navigation data.
"""
from math import atan2, sqrt, cos, sin, atan, pi

import rospy
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float64, Bool, Int16, Byte, Float32MultiArray
import time

READY_LINEAR = 0
READY = 1
IN_PROGRESS = 2

KLigne = -19.0
KAngle = 0.1

ANGLE_PRECISION = 0.03
KP_R = -0.20
KI_R = -0.07

GOTO_DELTA = -0.02
POSITION_SHIFT = 0.0

GOTO_BASE_DISTANCE_THRESHOLD = 0.02

ROTATE_TIME = 1.0

WHEEL_FORWARD_SPEED = 0.25
WHEEL_BACKWARD_SPEED = 0.19
WHEEL_TURN_SPEED_FORWARD = 0.30
WHEEL_TURN_SPEED_BACKWARD = 0.30

WHEEL_HIGH_SPEED_FACTOR = 2.0

EMERGENCY_FRONT = 1
EMERGENCY_BACK = 2
EMERGENCY_BOTH = 3
NO_EMERGENCY = 0

FRONT_BUMPERS = [0, 1]
BACK_BUMPERS = [2, 3]

NO_AXIS_MODE = 0
X_PLUS = 1
X_MINUS = 2
Y_PLUS = 3
Y_MINUS = 4
PREFERRED_AXIS = 5


class Kr1bou:
    def __init__(self):
        self.state = READY

        self.x = 0
        self.y = 0
        self.theta = 0

        self.objectif_x = 0.25
        self.objectif_y = 1.75
        self.objectif_theta = pi / 2

        self.vitesse_gauche = 0
        self.vitesse_droite = 0

        self.last_left_speed = 0
        self.last_right_speed = 0

        self.destination_angle = 0
        self.epsilon = 0.07

        self.force_forward = False
        self.force_backward = False
        self.axis_mode = NO_AXIS_MODE

        self.angle_to_follow = 0

        self.emergency_current = EMERGENCY_BACK
        self.bumper_emergency_in_progress = False
        self.bumpers = 0
        self.previous_objectif = [0, 0, 0, False, False] # (x, y, z, force_forwarde, force_backward) in case of emergency stop we will go back from 5 cm and com back to aboard objectif

        self.freq = rospy.get_param('/frequency')

        self.publisher_speed = rospy.Publisher('motor_speed', Vector3, queue_size=1)
        self.publisher_state = rospy.Publisher('state', Int16, queue_size=1)
        self.publisher_request_camera_update = rospy.Publisher('request_camera_update', Bool, queue_size=1)

        self.solar_mode = False

        self.integral_rotation = 0
        self.time_last_rotation = time.time()

        self.camera_position = Pose2D()

        rospy.Subscriber('odometry', Pose2D, self.update_pose)
        rospy.Subscriber('next_objectif', Pose2D, self.set_objectif)
        rospy.Subscriber('max_speed', Float64, set_max_speed)
        rospy.Subscriber('stop', Bool, self.stop)
        rospy.Subscriber('direction', Int16, self.update_moving_direction)
        rospy.Subscriber('emergency_stop', Int16, self.stop_move)
        rospy.Subscriber('solar_mode', Bool, self.update_solar_mode)
        rospy.Subscriber('axis_mode', Int16, self.update_axis_mode)
        rospy.Subscriber('bumper', Byte, self.update_bumpers)

    def stop_move(self, data: Int16):
        self.emergency_current = data.data

    def publish_speed(self):
        if self.state == IN_PROGRESS:
            self.update_speed()
        elif self.state == READY_LINEAR and self.objectif_theta != -1:
            self.update_rotation_speed()
        elif self.state == READY_LINEAR:
            if self.bumper_emergency_in_progress:
                rospy.loginfo("END EMERGENCY BACKARD")
                self.bumper_emergency_in_progress = False
                self.objectif_x, self.objectif_y, self.objectif_theta, self.force_forward, self.force_backward = self.previous_objectif
                data = Vector3()
                data.x, data.y = 0, 0
                self.publisher_speed.publish(data)
                self.publisher_request_camera_update.publish(Bool(True))
                self.state = IN_PROGRESS
                self.publish_state()
                time.sleep(1)
            else : 
                self.state = READY
                self.publish_state()
                self.vitesse_gauche = 0
                self.vitesse_droite = 0
        else:
            self.vitesse_gauche = 0
            self.vitesse_droite = 0
        data = Vector3()
        data.x = self.vitesse_gauche
        data.y = self.vitesse_droite

        if (data.x > 0 or data.y > 0) and self.emergency_current == EMERGENCY_FRONT :
            data.x = 0
            data.y = 0
        elif ((data.x < 0 or data.y < 0) and not self.solar_mode) and self.emergency_current == EMERGENCY_BACK:
            data.x = 0
            data.y = 0
        elif self.emergency_current == EMERGENCY_BOTH:
            data.x = 0
            data.y = 0

        self.publisher_speed.publish(data)

    def update_rotation_speed(self):
        angle_diff = angleDiffRad(self.objectif_theta, self.theta)
        self.integral_rotation += angle_diff*(time.time()-self.time_last_rotation)
        self.time_last_rotation = time.time()
        w = angle_diff * KP_R+self.integral_rotation*KI_R
        if abs(angle_diff) < ANGLE_PRECISION:
            self.state = READY
            self.publish_state()
            w = 0
        self.vitesse_gauche = w
        self.vitesse_droite = -w

    def update_solar_mode(self, data: Bool):
        self.solar_mode = data.data

    def update_axis_mode(self, data: Int16):
        self.axis_mode = data.data
        if self.axis_mode == X_PLUS:
            self.angle_to_follow = 0
        elif self.axis_mode == X_MINUS:
            self.angle_to_follow = pi
        elif self.axis_mode == Y_PLUS:
            self.angle_to_follow = pi / 2
        elif self.axis_mode == Y_MINUS:
            self.angle_to_follow = 3*pi / 2
        elif self.axis_mode == PREFERRED_AXIS:
            self.angle_to_follow = atan2(self.objectif_y - self.y, self.objectif_x - self.x)

    def update_pose(self, data: Pose2D):
        self.x = data.x
        self.y = data.y
        self.theta = data.theta

    def update_bumpers(self, data: Byte):
        self.bumpers = data.data
        if self.is_activated_bumper(FRONT_BUMPERS) and not(self.emergency_current in [EMERGENCY_BACK, EMERGENCY_BOTH]) and self.bumper_emergency_in_progress == False:
            self.state = IN_PROGRESS
            self.previous_objectif = [self.objectif_x, self.objectif_y, self.objectif_theta, self.force_forward, self.force_backward]
            self.bumper_emergency_in_progress = True
            self.objectif_x = self.x - 0.15*cos(self.theta)
            self.objectif_y = self.y - 0.15*sin(self.theta)
            self.objectif_theta = -1
            self.force_backward = True
            self.force_forward = False
            rospy.logdebug(f"(MOTION CONTROL) FRONT BUMPER IN BACKWARD MODE")

    def is_activated_bumper(self, ids):
        """Return True if all ids of bumpers are activated."""
        if any(self.bumpers & (1 << i) for i in ids):
            return True
        return False

    def update_speed(self, force_angle_line=-1):
        x2 = self.objectif_x
        y2 = self.objectif_y
        # rospy.loginfo(f"(MOTION CONTROL) from : {self.x} ; {self.y} to : {x2} ; {y2}")

        # rospy.loginfo(f"(MOTION CONTROL) going to : {x2} ; {y2}")
        force_forward = self.force_forward
        force_backward = self.force_backward

        if force_angle_line != -1:
            self.destination_angle = force_angle_line
        elif sqrt((self.x - x2) ** 2 + (self.y - y2) ** 2) > self.epsilon:
            self.destination_angle = atan2(y2 - self.y, x2 - self.x)
        
        destination_angle = self.destination_angle

        pos = [self.x, self.y]

        pos[0] -= cos(self.theta) * POSITION_SHIFT
        pos[1] -= sin(self.theta) * POSITION_SHIFT

        p1 = [x2, y2]
        p2 = [x2 + cos(destination_angle), y2 + sin(destination_angle)]

        dist_to_line = abs((p2[0] - p1[0]) * (p1[1] - pos[1]) - (p1[0] - pos[0]) * (p2[1] - p1[1]) / sqrt(
            (p2[0] - p1[0]) * (p2[0] - p1[0]) + (p2[1] - p1[1]) * (p2[1] - p1[1])))

        angle_line_to_robot = atan2(pos[1] - p1[1], pos[0] - p1[0])

        diff_angle_line_to_robot = angleDiffRad(destination_angle, angle_line_to_robot)

        is_on_right_side = diff_angle_line_to_robot > 0.0
        target_angle = destination_angle + atan(dist_to_line * 3.0) * (-1 if is_on_right_side else 1)  # dist_to_line * 3.5

        dx_base = x2 - pos[0]
        dy_base = y2 - pos[1]

        dist_to_base = sqrt(dx_base * dx_base + dy_base * dy_base)
        # rospy.loginfo(f"(MOTION CONTROL) 2 from : {self.x} ; {self.y} to : {x2} ; {y2}")
        # rospy.loginfo(f"(MOTION CONTROL) dx_base : {dx_base} / dy_base : {dy_base}")
        # rospy.loginfo(f"(MOTION CONTROL) dist_to_base : {dist_to_base}")
        
        m_goto_base_reached = False
        if dist_to_base < GOTO_BASE_DISTANCE_THRESHOLD:
            m_goto_base_reached = True
            self.state = READY_LINEAR
            self.integral_rotation = 0
            self.time_last_rotation = time.time()
            self.publish_state()

        angle_diff = angleDiffRad(target_angle, self.theta)
        backward = abs(angle_diff) > pi / 2

        backward = (backward or force_backward) and not force_forward
        if backward:
            angle_diff = angleDiffRad(target_angle + pi, self.theta)

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

        if self.state == READY_LINEAR:
            left_speed = 0
            right_speed = 0

        self.vitesse_gauche = right_speed
        self.vitesse_droite = left_speed

        # self.last_left_speed = left_speed
        # self.last_right_speed = right_speed

    def set_objectif(self, data: Pose2D):
        # rospy.loginfo(f"(MOTION CONTROL) New objective set to ({data.x};{data.y};{data.theta})")
        self.state = IN_PROGRESS
        self.publish_state()
        if self.bumper_emergency_in_progress:
            self.previous_objectif[0] = data.x
            self.previous_objectif[1] = data.y
            self.previous_objectif[2] = data.theta
        else :
            self.objectif_x = data.x
            self.objectif_y = data.y
            self.objectif_theta = data.theta

        if self.axis_mode == PREFERRED_AXIS:
            self.angle_to_follow = atan2(self.objectif_y - self.y, self.objectif_x - self.x)
        
        if self.objectif_x == -1 and self.objectif_y == -1:
            self.state = READY_LINEAR
            self.time_last_rotation = time.time()

    def stop(self, data: Bool):
        if data.data:
            self.state = READY
            self.publish_state()
            self.vitesse_gauche = 0
            self.vitesse_droite = 0
        else:
            self.state = IN_PROGRESS
            self.publish_state()

    def publish_state(self):
        temp = Int16()
        temp.data = self.state
        self.publisher_state.publish(temp)

    def update_moving_direction(self, data: Int16):
        if self.bumper_emergency_in_progress :
            self.previous_objectif[3] = data.data == 1
            self.previous_objectif[4] = data.data == -1
        else :
            self.force_forward = data.data == 1
            self.force_backward = data.data == -1

def angleDiffRad(from_a, to_a):
    return atan2(sin(to_a - from_a), cos(to_a - from_a))


def set_max_speed(data: Float64):
    global WHEEL_FORWARD_SPEED, WHEEL_BACKWARD_SPEED, WHEEL_TURN_SPEED_BACKWARD, WHEEL_TURN_SPEED_FORWARD
    WHEEL_FORWARD_SPEED = data.data
    WHEEL_BACKWARD_SPEED = data.data*.75
    WHEEL_TURN_SPEED_FORWARD = data.data+0.05
    WHEEL_TURN_SPEED_BACKWARD = data.data*.75+0.05
    if WHEEL_TURN_SPEED_BACKWARD > 0.5:
        WHEEL_TURN_SPEED_BACKWARD = 0.5
    if WHEEL_TURN_SPEED_FORWARD > 0.5:
        WHEEL_TURN_SPEED_FORWARD = 0.5
    # rospy.loginfo(f"(MOTION CONTROL) New speed set to {data.data}")


def run(data: Bool):
    global start
    start = data.data
    rospy.loginfo(f"{rospy.get_name()} received {start} from running_phase")


if __name__ == "__main__":
    start = False
    try:
        # Initialization
        rospy.init_node('controller', anonymous=True)
        rospy.loginfo("[START] Controller node has started.")
        # Wait for the running_phase True signal
        rate = rospy.Rate(rospy.get_param('/frequency'))
        rospy.Subscriber('running_phase', Bool, run)
        robot = Kr1bou()
        while not start:
            rate.sleep()

        rospy.sleep(0.2)
        while not rospy.is_shutdown():
            robot.publish_speed()
            rate.sleep()
    finally:
        rospy.loginfo("[STOP] Controller node has stopped.")
