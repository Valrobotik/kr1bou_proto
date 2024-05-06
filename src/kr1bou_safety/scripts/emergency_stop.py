#!/usr/bin/env python3
"""
Looks out for Emergency. Stops everything.
"""

import rospy
from std_msgs.msg import Int16, Byte, Float32MultiArray, Bool
from geometry_msgs.msg import Pose2D, PoseArray, Pose

import math

EMERGENCY_FRONT = 1
EMERGENCY_BACK = 2
EMERGENCY_BOTH = 3
NO_EMERGENCY = 0

TEAM_BLUE = 1
TEAM_YELLOW = 0

robot_position = Pose2D()
def callback_robot_position(data):
    global robot_position
    robot_position = data

lidar_obstacles = []
def callback_lidar_obstacles(data: PoseArray):
    global lidar_obstacles
    lidar_obstacles = data.poses

bumper_1_front = False
bumper_2_front = False
bumper_3_back = False
bumper_4_back = False
def callback_bumper(data:Byte):
    global bumper_1_front, bumper_2_front, bumper_3_back, bumper_4_back
    bumper_1_front = data.data & 0b0001
    bumper_2_front = data.data & 0b0010
    bumper_3_back = data.data & 0b0100
    bumper_4_back = data.data & 0b1000

team_color = -1
def callback_team_color(data:Bool):
    global team_color
    team_color = data.data

camera_adverse_position = Pose2D()
camera_adverse_position.x = -1
camera_adverse_position.y = -1
camera_adverse_position.theta = -1
def callback_camera(data:Float32MultiArray):
    if team_color == -1:
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

US_obstacles = []
def callback_US_data(data:Float32MultiArray):
    pass ### TODO

def list_of_obstacles():
    obstacles = []
    for obstacle in lidar_obstacles:
        obstacles.append((obstacle.position.x, obstacle.position.y))

    if camera_adverse_position.x != -1 and camera_adverse_position.y != -1:
        obstacles.append((camera_adverse_position.x, camera_adverse_position.y))

    for obstacle in US_obstacles:
        obstacles.append((obstacle[0], obstacle[1]))

    for i in range(len(obstacles)):
        rospy.loginfo(f"ROBOT POSITION: ({robot_position.x}, {robot_position.y}, {robot_position.theta})")
        rospy.loginfo(f"Obstacle absolue: {obstacles[i]}")
        obstacles[i] = ((obstacles[i][0] - robot_position.x)*math.cos(robot_position.theta), (obstacles[i][1] - robot_position.y)*math.sin(robot_position.theta))
        rospy.loginfo(f"Obstacle relatif: {obstacles[i]}")
    return obstacles
    

def emergency_stop_run():
    global robot_position, lidar_obstacles, team_color, camera_adverse_position, US_obstacles, bumper_1_front, bumper_2_front, bumper_3_back, bumper_4_back
    rate = rospy.Rate(10) # 10hz
    emergency_state = NO_EMERGENCY
    while not rospy.is_shutdown():
        obstacles = list_of_obstacles()
        emergency_front = False
        emergency_back = False
        for obstacle in obstacles:
            if obstacle[0]>0.0 and obstacle[0]<0.25 and obstacle[1]>-0.16 and obstacle[1]<0.16:
                emergency_front = True
            if obstacle[0]>-0.25 and obstacle[0]<0.0 and obstacle[1]>-0.16 and obstacle[1]<0.16:
                emergency_back = True
        if bumper_1_front or bumper_2_front:
            emergency_front = True
        if bumper_3_back or bumper_4_back:
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

    ## Subscribers
    rospy.Subscriber('odometry', Pose2D, callback_robot_position)
    rospy.Subscriber('lidar_data', PoseArray, callback_lidar_obstacles)
    rospy.Subscriber('bumper', Byte, callback_bumper)
    rospy.Subscriber('team', Bool, callback_team_color)
    rospy.Subscriber('camera', Float32MultiArray, callback_camera)
    rospy.Subscriber('ultrasound_sensor_data', Float32MultiArray, callback_US_data)
    
    emergency_stop_run()