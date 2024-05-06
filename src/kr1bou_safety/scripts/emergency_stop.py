#!/usr/bin/env python3
"""
Looks out for Emergency. Stops everything.
"""

import rospy
from std_msgs.msg import Int16, Byte, Float32MultiArray, Bool
from geometry_msgs.msg import Pose2D, PoseArray

import math

EMERGENCY_FRONT = 1
EMERGENCY_BACK = 2
EMERGENCY_BOTH = 3
NO_EMERGENCY = 0

TEAM_BLUE = 1
TEAM_YELLOW = 0

robot_position = Pose2D()
lidar_obstacles = []

bumpers = 0
team_color = -1

camera_adverse_position = Pose2D()
camera_adverse_position.x = -1
camera_adverse_position.y = -1
camera_adverse_position.theta = -1

US_obstacles = []


def callback_robot_position(data):
    global robot_position
    robot_position = data


def callback_lidar_obstacles(data: PoseArray):
    global lidar_obstacles
    lidar_obstacles = data.poses


def callback_bumper(data: Byte):
    global bumpers
    bumpers = data.data


def callback_team_color(data: Bool):
    global team_color
    team_color = data.data


def callback_camera(data: Float32MultiArray):
    if team_color == -1:
        rospy.logwarn("Team color not set")
        return
    blue_robot_x, blue_robot_y, blue_robot_theta, yellow_robot_x, yellow_robot_y, yellow_robot_theta = data.data
    if team_color == TEAM_YELLOW:
        camera_adverse_position.x = blue_robot_x
        camera_adverse_position.y = blue_robot_y
        camera_adverse_position.theta = blue_robot_theta
    else:
        camera_adverse_position.x = yellow_robot_x
        camera_adverse_position.y = yellow_robot_y
        camera_adverse_position.theta = yellow_robot_theta


def callback_US_data(data: Float32MultiArray):
    global US_obstacles
    US_obstacles = [(data.data[i], data.data[i + 1]) for i in range(0, len(data.data), 2)]

def is_activated_bumper(ids):
    """Return True if all ids of bumpers are activated."""
    if any(bumpers & (1 << i) for i in ids):
        return True
    return False

def log_debug_obstacles(obstacles):
    for obstacle in obstacles:
        dx = obstacle[0] - robot_position.x
        dy = obstacle[1] - robot_position.y
        alpha = robot_position.theta
        rospy.loginfo(f"ROBOT POSITION: ({robot_position.x}, {robot_position.y}, {robot_position.theta})")
        rospy.loginfo(f"Obstacle absolue: ({obstacle[0]}, {obstacle[1]})")
        rospy.loginfo(f"Obstacle relatif: ({dx * math.cos(alpha) + dy * math.sin(alpha)}, {-dx * math.sin(alpha) + dy * math.cos(alpha)})")


def list_of_obstacles():
    obstacles = []
    for obstacle in lidar_obstacles:
        obstacles.append((obstacle.position.x, obstacle.position.y))

    if camera_adverse_position.x != -1 and camera_adverse_position.y != -1:
        obstacles.append((camera_adverse_position.x, camera_adverse_position.y))

    obstacles.extend(US_obstacles)

    log_debug_obstacles(US_obstacles)

    for i in range(len(obstacles)):
        # rospy.loginfo(f"ROBOT POSITION: ({robot_position.x}, {robot_position.y}, {robot_position.theta})")
        # rospy.loginfo(f"Obstacle absolue: {obstacles[i]}")
        dx = obstacles[i][0] - robot_position.x
        dy = obstacles[i][1] - robot_position.y
        alpha = robot_position.theta
        obstacles[i] = (dx * math.cos(alpha) + dy * math.sin(alpha), -dx * math.sin(alpha) + dy * math.cos(alpha))
    return obstacles


def emergency_stop_run():
    global robot_position, lidar_obstacles, team_color, camera_adverse_position, US_obstacles, bumper_1_front, bumper_2_front, bumper_3_back, bumper_4_back
    rate = rospy.Rate(10)  # 10hz
    emergency_state = NO_EMERGENCY
    while not rospy.is_shutdown():
        obstacles = list_of_obstacles()
        emergency_front = False
        emergency_back = False
        for obstacle in obstacles:
            if 0.0 < obstacle[0] < 0.40 and -0.22 < obstacle[1] < 0.22:
                emergency_front = True
            if -0.50 < obstacle[0] < 0.0 and -0.22 < obstacle[1] < 0.22:
                emergency_back = True
        if is_activated_bumper([0, 1]):
            emergency_front = True
        if is_activated_bumper([2, 3]):
            emergency_back = True
        if emergency_front and emergency_back:
            if emergency_state != EMERGENCY_BOTH:
                rospy.logwarn("EMERGENCY STOP: Both")
                emergency_state = EMERGENCY_BOTH
                pub.publish(EMERGENCY_BOTH)
        elif emergency_front:
            if emergency_state != EMERGENCY_FRONT:
                rospy.logwarn("EMERGENCY STOP: Front")
                emergency_state = EMERGENCY_FRONT
                pub.publish(EMERGENCY_FRONT)
        elif emergency_back:
            if emergency_state != EMERGENCY_BACK:
                rospy.logwarn("EMERGENCY STOP: Back")
                emergency_state = EMERGENCY_BACK
                pub.publish(EMERGENCY_BACK)
        else:
            if emergency_state != NO_EMERGENCY:
                rospy.logwarn("NO EMERGENCY")
                emergency_state = NO_EMERGENCY
            pub.publish(NO_EMERGENCY)

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('emergency_stop')
    pub = rospy.Publisher('emergency_stop', Int16, queue_size=10)

    # Subscribers
    rospy.Subscriber('odometry', Pose2D, callback_robot_position)
    rospy.Subscriber('lidar_data', PoseArray, callback_lidar_obstacles)
    rospy.Subscriber('bumper', Byte, callback_bumper)
    rospy.Subscriber('team', Bool, callback_team_color)
    rospy.Subscriber('camera', Float32MultiArray, callback_camera)
    rospy.Subscriber('ultrasound_sensor_data', Float32MultiArray, callback_US_data)

    emergency_stop_run()
