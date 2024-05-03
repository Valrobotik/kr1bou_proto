#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64, Bool, Int8, Int16, Float32MultiArray, Byte

import time
from math import sqrt, pi

from search_path import Node, a_star, clean_path
from utils import setup_maze, is_path_valid, Objective

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
        self.need_for_compute = True  # Whether to ask for a new path
        self.next_pos_obj = [0, 0, 0]  # Next position to go to / Intermediate objective
        self.current_objective: Objective = Objective(0, 0, 0, 0)  # Current objective

        # -- Map/Graph related --
        self.map_boundaries = [int(m) for m in rospy.get_param('/map_boundaries')]
        self.resolution = rospy.get_param('/resolution')  # Resolution to centimeters for example.
        self.maze = [[Node((x, y), 0) for y in range(int(self.map_boundaries[3] * self.resolution))]
                     for x in range(int(self.map_boundaries[2] * self.resolution))]
        self.path = []  # List of waypoints to follow
        self.obstacles = []  # List of obstacles

        self.custom_waiting_rate = rospy.Rate(20)

        # -- Subscribers --
        self.position = Pose2D()  # Get the initial position of the robot
        # Get the ultrasound sensor data
        self.us_data = [(-1, -1), (-1, -1), (-1, -1), (-1, -1), (-1, -1), (-1, -1), (-1, -1), (-1, -1), (-1, -1),
                        (-1, -1)]
        # Get the back camera data
        self.latest_solar_winner = 0
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
            if self.need_for_compute:  # New sensor data
                self.close_enough_to_waypoint()  # Remove the waypoint if the robot is close enough
                rospy.loginfo(f"(STRATEGY) Objectives : {self.objectives}")
                self.compute_path()
                rospy.loginfo(f"(STRATEGY) Path : {self.path}")
                self.need_for_compute = False
                self.follow_path()
            else:
                self.wait_until_ready()

    def close_enough_to_waypoint(self, threshold=5.0):
        while (len(self.path) > 0 and sqrt((self.position.x - self.path[0].position[0]) ** 2 +
                                           (self.position.y - self.path[0].position[1]) ** 2)
               < threshold / self.resolution):
            rospy.loginfo(f"""(STRATEGY) Distance: {sqrt((self.position.x - self.path[0].position[0]) ** 2 +
                                                         (self.position.y - self.path[0].position[1]) ** 2)}""")
            rospy.loginfo(f"(STRATEGY) Threshold: {threshold / self.resolution}")
            rospy.loginfo(f"""(STRATEGY) Robot is close enough to the nearest waypoint. Removing {self.path[0]} 
            from the path.""")
            self.path.pop(0)  # Remove if he is close enough to the current intermediate objective

    def compute_path(self):
        """Aggregate all the data and compute the path to follow using A* algorithm. Neighbors are defined by a dict of
        the form {direction: (cost, neighbor_node)}. The cost is very high if the neighbor is an obstacle.
        :return: the path to follow
        """
        rospy.loginfo("(STRATEGY) IN COMPUTE PATH FUNCTION")
        self.maze = setup_maze(self.maze, self.us_data, self.resolution)
        # Get the start and end nodes
        origin = self.maze[int(self.position.x * self.resolution)][int(self.position.y * self.resolution)]
        origin.orientation = self.position.theta
        if self.path == [] and self.objectives != []:  # Get new closest objective
            self.current_objective = self.objectives[0]
            self.objectives.pop(0)

        rospy.loginfo(f"(STRATEGY) Current start/end : {origin.position}/{self.current_objective}")
        if is_path_valid(self.path, self.us_data, self.resolution):  # Check if the path is still valid
            rospy.loginfo("(STRATEGY) Path still exists")
        else:  # Compute a new path
            rospy.loginfo(f"(STRATEGY) Computing path from {origin.position} to {self.current_objective}")
            onset = time.time()
            path = a_star(origin, self.maze[int(self.current_objective.x * self.resolution)]
                                           [int(self.current_objective.y * self.resolution)])
            path = clean_path(path)
            self.path = [
                Node((node.position[0] / self.resolution, node.position[1] / self.resolution), node.orientation) for
                node in path]
            rospy.loginfo(f"(STRATEGY) Path computed in {time.time() - onset} seconds")
            rospy.loginfo(f"(STRATEGY) Converted path : {self.path}")

        # Remove node if the robot is already on it if the robot is already following a path
        self.close_enough_to_waypoint(threshold=7.0)

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
            rospy.loginfo("(STRATEGY) Waiting for the robot to be ready...")
            self.custom_waiting_rate.sleep()
            if (sqrt((self.next_pos_obj[0] - self.position.x) ** 2 + (
                    self.next_pos_obj[1] - self.position.y) ** 2) < 0.07 and self.next_pos_obj[2] == -1):
                rospy.loginfo("(STRATEGY) Robot is close enough to the node. Waiting for the next order.")
                rospy.loginfo(f"""(STRATEGY) Current dist: {sqrt((self.next_pos_obj[0] - self.position.x) ** 2 + 
                                                                 (self.next_pos_obj[1] - self.position.y) ** 2)}""")
                rospy.loginfo(f"(STRATEGY) Threshold : 0.07")
                break
        self.need_for_compute = True

    def follow_path(self):
        if self.path and len(self.path) != 0:
            rospy.loginfo(f"(STRATEGY) Following path : {self.path}")
            self.go_to(self.path[0].position[0], self.path[0].position[1], -1, DEFAULT_MAX_SPEED, BEST_DIRECTION)
            rospy.loginfo(f"(STRATEGY) Going to {self.path[0]}")
        else:
            rospy.loginfo("(STRATEGY) No path found")
            
    def setup_subscribers(self):
        rospy.Subscriber('odometry', Pose2D, self.update_position)
        rospy.Subscriber('ultrasound_sensor_data', Float32MultiArray, self.update_us_data)
        rospy.Subscriber('camera', Pose2D, self.update_camera)
        rospy.Subscriber('bumper', Byte, self.update_bumpers)
        rospy.Subscriber('state', Int16, self.update_state)
        rospy.Subscriber('Team', Bool, self.update_team)

    def reset_position_from_camera(self):
        """Publishes the camera position to the odometry topic to correct the odometry"""
        rospy.loginfo("(STRATEGY) Debug odom correction")
        if time.time() - self.last_time_cam < 10:
            self.got_cam_data = False
            while not self.got_cam_data:
                rospy.sleep(0.05)
            self.publisher_correct_odom.publish(self.camera_position)
            rospy.loginfo("(STRATEGY) Odometry corrected")
        else:
            rospy.logwarn("(STRATEGY) No connexion with camera")
        self.need_rst_odom = False
        self.got_cam_data = False

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
        self.need_for_compute = True

    def update_us_data(self, data):
        raw = data.data
        # Unflatten the list of tuples ->
        # [(sensor1_reading_x, sensor1_reading_y), ..., (sensorN_reading_x, sensorN_reading_y)] #cm
        self.us_data = [(raw[i], raw[i + 1]) for i in range(0, len(raw), 2)]
        self.need_for_compute = True

    def update_camera(self, data: Pose2D):
        """Updates the info from the camera"""
        rospy.loginfo("(STRATEGY) Camera received")
        self.camera_position = data
        if self.camera_position.theta < 0:
            self.camera_position.theta = self.camera_position.theta + 2 * pi
        self.camera_position.theta = 2 * pi - self.camera_position.theta
        self.got_cam_data = True

        rospy.loginfo(f"(STRATEGY) {self.camera_position}")

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
        strategy_manager.solar_pub.publish(Int16(180))
        strategy_manager.run()
    finally:
        rospy.loginfo("[STOP] Strategy node has stopped.")
