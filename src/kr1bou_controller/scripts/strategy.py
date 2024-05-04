#!/usr/bin/env python3

import time
from math import sqrt

from geometry_msgs.msg import Pose2D, PoseArray
from std_msgs.msg import Float64, Bool, Int8, Int16, Float32MultiArray, Byte

from search_path import a_star, clean_path
from utils import *

READY_LINEAR = 0
READY = 1
IN_PROGRESS = 2

FORWARD = 1
BACKWARD = -1
BEST_DIRECTION = 0

TEAM_BLUE = 1
TEAM_YELLOW = 0

DEFAULT_MAX_SPEED = 0.25


class Strategy:
    def __init__(self) -> None:
        # -- Robot related --
        self.need_for_compute = True  # Whether to ask for a new path
        self.next_pos_obj = [0, 0, 0]  # Next position to go to / Intermediate objective

        # -- Map/Graph related --
        self.map_boundaries = [int(m) for m in rospy.get_param('/map_boundaries')]
        self.resolution = rospy.get_param('/resolution')  # Resolution to centimeters for example.
        self.path = []  # List of waypoints to follow
        self.obstacles = set()  # List of obstacles
        self.previous_obstacles = set()  # Previous obstacles
        self.maze = np.zeros(
            (int(self.map_boundaries[2] * self.resolution), int(self.map_boundaries[3] * self.resolution)), dtype=Node)
        self.maze = setup_maze(self.maze, self.obstacles)
        self.custom_waiting_rate = rospy.Rate(20)

        # -- Subscribers --
        self.position = Pose2D()  # Get the initial position of the robot
        self.enemy_position = Pose2D()  # Get the position of the enemy robot
        self.lidar_data = None  # Get the lidar data
        # Get the ultrasound sensor data
        self.us_data = [(-1, -1) for _ in range(10)]
        self.latest_solar_winner = 0  # Get the back camera data
        # Get the camera data
        self.need_rst_odom = False
        self.last_time_cam = time.time()
        self.camera_position = Pose2D()
        self.got_cam_data = False
        # Bumpers
        for i in range(1, 5):
            setattr(self, f'bumper_{i}', False)
        # State of the robot
        self.state_robot = READY
        # Team Color
        self.team = -1
        self.setup_subscribers()

        # Set up the objectives
        self.objectives = [Objective(x, y, theta, sqrt((x - self.position.x) ** 2 + (y - self.position.y) ** 2)) for
                           x, y, theta in rospy.get_param('/objectives')]
        self.current_objective = self.objectives[0]

        # -- Publishers --
        self.solar_pub = rospy.Publisher('solar_angle', Int16, queue_size=1)
        self.pos_ordre_pub = rospy.Publisher('next_objectif', Pose2D, queue_size=1)
        self.direction_pub = rospy.Publisher('direction', Int16, queue_size=1)
        self.speed_ctrl_pub = rospy.Publisher('max_speed', Float64, queue_size=1)
        self.publisher_correct_odom = rospy.Publisher('odom_corrected', Pose2D, queue_size=1)

    def run(self):
        rospy.loginfo("(STRATEGY) Strategy running loop has started.")
        while self.team == -1 and not rospy.is_shutdown():
            rospy.sleep(0.05)
        while not rospy.is_shutdown():
            self.close_enough_to_waypoint()  # Remove the waypoint if the robot is close enough
            if not self.objectives and not self.path:
                break
            self.compute_path()
            # rospy.loginfo(f"(STRATEGY) Path : {self.path}")
            self.follow_path()

    def close_enough_to_waypoint(self, threshold=5.0):
        while (len(self.path) > 0 and sqrt((self.position.x - self.path[0].position[0]) ** 2 +
                                           (self.position.y - self.path[0].position[1]) ** 2)
               < threshold / self.resolution):
            # rospy.loginfo(f"""(STRATEGY) Distance: {sqrt((self.position.x - self.path[0].position[0]) ** 2 +
            #                                              (self.position.y - self.path[0].position[1]) ** 2)}""")
            # rospy.loginfo(f"(STRATEGY) Threshold: {threshold / self.resolution}")
            # rospy.loginfo(f"""(STRATEGY) Robot is close enough to the nearest waypoint. Removing {self.path[0]} 
            # from the path.""")
            self.path.pop(0)  # Remove if he is close enough to the current intermediate objective

    def compute_path(self):
        """Aggregate all the data and compute the path to follow using A* algorithm. Neighbors are defined by a dict of
        the form {direction: (cost, neighbor_node)}. The cost is very high if the neighbor is an obstacle.
        :return: the path to follow
        """
        self.obstacles = set(get_discrete_obstacles(self.lidar_data, self.us_data, self.resolution))
        self.maze = update_maze(self.maze, self.previous_obstacles, self.obstacles)
        self.previous_obstacles = self.obstacles

        if self.path == [] and self.objectives != []:  # Get new closest objective
            self.reset_position_from_camera()
            self.current_objective = self.objectives[0]
            self.objectives.pop(0)
            rospy.loginfo(f"(STRATEGY) New objective : {self.current_objective}")
            rospy.loginfo(f"(STRATEGY) Remaining objectives : {self.objectives}")

        # Get the start and end nodes
        origin = self.maze[int(self.position.x * self.resolution)][int(self.position.y * self.resolution)]
        origin.orientation = self.position.theta

        # rospy.loginfo(f"(STRATEGY) Current start/end : {origin.position}/{self.current_objective}")
        if is_path_valid(self.path, self.obstacles):  # Check if the path is still valid
            rospy.loginfo("(STRATEGY) Path still exists")
        else:  # Compute a new path
            # rospy.loginfo(f"(STRATEGY) Computing path from {origin.position} to {self.current_objective}")
            path = a_star(origin, self.maze[int(self.current_objective.x * self.resolution)][
                int(self.current_objective.y * self.resolution)])
            path = clean_path(path)
            self.path = [
                Node((node.position[0] / self.resolution, node.position[1] / self.resolution), node.orientation) for
                node in path]
            # rospy.loginfo(f"(STRATEGY) Path computed in {time.time() - onset} seconds")
            # rospy.loginfo(f"(STRATEGY) Converted path : {self.path}")

        # Remove node if the robot is already on it if the robot is already following a path
        self.close_enough_to_waypoint(threshold=4.0)

    def go_to(self, x=-1, y=-1, alpha=-1, speed=0.25, direction=0):
        """go to position (x, y, alpha)
        -> if alpha = -1 go to (x,y)
        -> direction = [0 : best option, 1 : forward, -1 : backward]"""
        obj = Pose2D()
        obj.x = x
        obj.y = y
        obj.theta = alpha
        self.next_pos_obj = [x, y, alpha]
        direction_data = Int16()
        direction_data.data = direction
        speed_data = Float64()
        speed_data.data = speed
        self.direction_pub.publish(direction_data)
        self.speed_ctrl_pub.publish(speed_data)
        self.pos_ordre_pub.publish(obj)

    def wait_until_ready(self):
        while self.state_robot != READY:
            # rospy.loginfo("(STRATEGY) Waiting for the robot to be ready...")
            self.custom_waiting_rate.sleep()

    def follow_path(self):
        if self.path:
            # rospy.loginfo(f"(STRATEGY) Following path : {self.path}")
            self.go_to(self.path[0].position[0], self.path[0].position[1], -1, DEFAULT_MAX_SPEED, BEST_DIRECTION)
            rospy.loginfo(f"(STRATEGY) Going to {self.path[0]}")
        else:
            rospy.loginfo("(STRATEGY) No path found")

    def setup_subscribers(self):
        rospy.Subscriber('odometry', Pose2D, self.update_position)
        rospy.Subscriber('lidar_data', PoseArray, self.update_lidar_data)
        rospy.Subscriber('ultrasound_sensor_data', Float32MultiArray, self.update_us_data)
        rospy.Subscriber('camera', Float32MultiArray, self.update_camera)
        rospy.Subscriber('bumper', Byte, self.update_bumpers)
        rospy.Subscriber('state', Int16, self.update_state)
        rospy.Subscriber('Team', Bool, self.update_team)

    def reset_position_from_camera(self):
        """Publishes the camera position to the odometry topic to correct the odometry"""
        rospy.loginfo("(STRATEGY) Debug odom correction")
        self.wait_until_ready()
        rospy.sleep(0.3)
        if time.time() - self.last_time_cam < 3:
            self.got_cam_data = False
            while not self.got_cam_data:
                rospy.sleep(0.05)
            self.publisher_correct_odom.publish(self.camera_position)
            rospy.loginfo("(STRATEGY) Odometry corrected")
        else:
            rospy.logwarn("(STRATEGY) No connexion with camera")
        self.need_rst_odom = False
        self.got_cam_data = False
        self.position = self.camera_position

    def update_bumpers(self, data: Byte):
        """Update the bumper states by reading the Byte message from the bumper topic."""
        data = int(data.data)
        for i in range(4):
            if data & 2 ** i:
                setattr(self, f'bumper_{i + 1}', True)
            else:
                setattr(self, f'bumper_{i + 1}', False)
        self.need_for_compute = True

    def update_position(self, data):
        self.position = data
        # rospy.loginfo(f"(STRATEGY) Position received : {self.position}")
        self.need_for_compute = True

    def update_lidar_data(self, data):
        self.lidar_data = data
        self.need_for_compute = True

    def update_us_data(self, data):
        raw = data.data
        self.us_data = [(raw[i], raw[i + 1]) for i in range(0, len(raw), 2)]  # Unflatten the data
        self.need_for_compute = True

    def update_camera(self, data: Float32MultiArray):
        """Updates the info from the camera [team_blue_x, team_blue_y, team_blue_theta, team_yellow_x, team_yellow_y,
        team_yellow_theta]"""

        if self.team == -1:
            return
        blue_robot = Pose2D()
        yellow_robot = Pose2D()
        blue_robot.x, blue_robot.y, blue_robot.theta, yellow_robot.x, yellow_robot.y, yellow_robot.theta = data.data

        # Convert to meters
        blue_robot.x /= 100
        blue_robot.y /= 100
        yellow_robot.x /= 100
        yellow_robot.y /= 100

        # rospy.loginfo(f"(STRATEGY) Camera - Blue robot : {blue_robot}")
        # rospy.loginfo(f"(STRATEGY) Camera - Yellow robot : {yellow_robot}")

        if self.team == TEAM_BLUE:  # discriminate between own robot and enemy robot
            self.camera_position, self.enemy_position = blue_robot, yellow_robot
        else:
            self.camera_position, self.enemy_position = yellow_robot, blue_robot
        self.camera_position.theta = clamp_theta(self.camera_position.theta)
        self.enemy_position.theta = clamp_theta(self.enemy_position.theta)

        self.got_cam_data = True
        self.last_time_cam = time.time()

    def update_team(self, data: Bool):
        global start
        if not start or self.team != -1:
            if data.data:
                self.team = TEAM_BLUE
            else:
                self.team = TEAM_YELLOW
        else:
            rospy.loginfo("(STRATEGY) YOU CAN'T CHANGE TEAM AFTER STARTING THE GAME !")

    def update_state(self, data: Int16):
        self.state_robot = data.data
        self.need_for_compute = True

    def update_solar_winner(self, winner: Int8):
        self.latest_solar_winner = winner.data

    def stop(self):
        self.go_to(self.position.x, self.position.y)  # Stop the robot

    def back_until_bumper(self, speed=0.15, axis='y+', direction=BACKWARD):
        while not self.bumper_1 and not self.bumper_2 and not self.bumper_3 and not self.bumper_4:
            if axis == 'y+':
                self.go_to(self.position.x, self.position.y + 2, speed=speed, direction=direction)
            elif axis == 'y-':
                self.go_to(self.position.x, self.position.y - 2, speed=speed, direction=direction)
            elif axis == 'x+':
                self.go_to(self.position.x + 3, self.position.y, speed=speed, direction=direction)
            elif axis == 'x-':
                self.go_to(self.position.x - 3, self.position.y, speed=speed, direction=direction)
        self.stop()

    def move_relative(self, distance, speed=0.20, axis='y-', direction=BEST_DIRECTION):
        if axis == 'y-':
            self.go_to(self.position.x, self.position.y - distance, speed=speed, direction=direction)
        elif axis == 'y+':
            self.go_to(self.position.x, self.position.y + distance, speed=speed, direction=direction)
        elif axis == 'x-':
            self.go_to(self.position.x - distance, self.position.y, speed=speed, direction=direction)
        elif axis == 'x+':
            self.go_to(self.position.x + distance, self.position.y, speed=speed, direction=direction)
        self.wait_until_ready()

    def solar_panel(self, id = 0):
        objective_x, objective_y = rospy.get_param('/solar_panel')[id]
        

def run(data):
    global start
    start = data
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from RunningPhase")


if __name__ == "__main__":
    try:
        start = False
        rospy.init_node("strategy")
        rospy.loginfo("[START] Strategy node has started.")
        strategy_manager = Strategy()

        rospy.Subscriber('runningPhase', Bool, run)
        rate = rospy.Rate(rospy.get_param('/frequency'))
        while not start:
            rate.sleep()

        strategy_manager.reset_position_from_camera()
        rospy.sleep(0.1)
        strategy_manager.run()
    finally:
        rospy.loginfo("[STOP] Strategy node has stopped.")
