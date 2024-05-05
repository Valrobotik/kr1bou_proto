#!/usr/bin/env python3

import time
from math import sqrt

import rospy
from geometry_msgs.msg import Pose2D, PoseArray
from std_msgs.msg import Float64, Bool, Int8, Int16, Float32MultiArray, Byte

from search_path import a_star, clean_path
from utils import *

import pickle

READY_LINEAR = 0
READY = 1
IN_PROGRESS = 2

FORWARD = 1
BACKWARD = -1
BEST_DIRECTION = 0

TEAM_BLUE = 1
TEAM_YELLOW = 0

SOLAR_BLUE = 1
SOLAR_YELLOW = -1
SOLAR_NEUTRAL = 0
SOLAR_BOTH = 2
SOLAR_DEFAULT = -2

DEFAULT_MAX_SPEED = 0.25

NO_AXIS_MODE = 0
X_PLUS = 1
X_MINUS = 2
Y_PLUS = 3
Y_MINUS = 4
PREFERRED_AXIS = 5


class Strategy:
    def __init__(self) -> None:
        # -- Robot related --
        self.need_for_compute = True  # Whether to ask for a new path
        self.next_pos_obj = [0, 0, 0]  # Next position to go to / Intermediate objective
        self.game_states = []

        # -- Map/Graph related --
        self.map_boundaries = [int(m) for m in rospy.get_param('/map_boundaries')]
        self.resolution = rospy.get_param('/resolution')  # Resolution to centimeters for example.
        self.radius = rospy.get_param('/radius')  # Radius of the robot in the resolution/unit given.
        self.raw_path = []  # Raw path to follow
        self.path = []  # List of waypoints to follow
        self.obstacles1 = set()  # List of obstacles with radius
        self.obstacles2 = set()  # List of obstacles with radius - 10
        self.previous_obstacles = set()  # Previous obstacles
        self.maze = np.zeros(
            (int(self.map_boundaries[2] * self.resolution), int(self.map_boundaries[3] * self.resolution)), dtype=Node)
        self.maze = setup_maze(self.maze, self.obstacles1)
        self.custom_waiting_rate = rospy.Rate(20)

        # -- Subscribers --
        self.position = Pose2D()  # Get the initial position of the robot
        self.enemy_position = Pose2D()  # Get the position of the enemy robot
        self.lidar_data = []  # Get the lidar data
        # Get the ultrasound sensor data
        self.us_data = [(-1, -1) for _ in range(10)]
        self.latest_solar_winner = SOLAR_DEFAULT
        self.need_solar_winner = False
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

        # -- Strategy related --
        self.objectives = []
        self.current_objective = None
        self.current_direction = BEST_DIRECTION
        self.start_time = time.time()

        # -- Publishers --
        self.solar_pub = rospy.Publisher('solar_angle', Int16, queue_size=1)
        self.pos_ordre_pub = rospy.Publisher('next_objectif', Pose2D, queue_size=1)
        self.direction_pub = rospy.Publisher('direction', Int16, queue_size=1)
        self.speed_ctrl_pub = rospy.Publisher('max_speed', Float64, queue_size=1)
        self.publisher_correct_odom = rospy.Publisher('odom_corrected', Pose2D, queue_size=1)
        self.solar_mode_pub = rospy.Publisher('solar_mode', Bool, queue_size=1)
        self.axis_mode_pub = rospy.Publisher('axis_mode', Int16, queue_size=1)

    def run(self):
        rospy.loginfo("(STRATEGY) Strategy running loop has started.")
        while self.team == -1 and not rospy.is_shutdown():
            rospy.sleep(0.05)

        self.debug_phase()
        #self.plant_phase()
        #self.solar_phase()
        self.home_phase()

        self.go_to(2.8, 1, on_axis=X_PLUS)
        rospy.loginfo("(STRATEGY) Strategy running loop has stopped.")

    # -- Phases --
    def debug_phase(self):
        rospy.loginfo("(STRATEGY) Starting debug phase")
        max_time = rospy.get_param("/phases/debug")
        if self.team == TEAM_BLUE:
            self.objectives = [
                Objective(x, y, theta, sqrt((x - self.position.x) ** 2 + (y - self.position.y) ** 2), direction) for
                x, y, theta, direction in rospy.get_param("/objectives/blue/debug")]
        else:
            self.objectives = [
                Objective(x, y, theta, sqrt((x - self.position.x) ** 2 + (y - self.position.y) ** 2), direction) for
                x, y, theta, direction in rospy.get_param("/objectives/yellow/debug")]
        self.current_objective = self.objectives[0]
        while (self.path or self.objectives) and max_time > time.time() - self.start_time:
            self.compute_path()
            self.follow_path()
        rospy.loginfo("(STRATEGY) Debug phase is over")

    def plant_phase(self):
        rospy.loginfo("(STRATEGY) Starting plant phase")
        max_time = rospy.get_param("/phases/plant")
        if self.team == TEAM_BLUE:
            self.objectives = [
                Objective(x, y, theta, sqrt((x - self.position.x) ** 2 + (y - self.position.y) ** 2), direction) for
                x, y, theta, direction in rospy.get_param("/objectives/blue/plant")]
        else:
            self.objectives = [
                Objective(x, y, theta, sqrt((x - self.position.x) ** 2 + (y - self.position.y) ** 2), direction) for
                x, y, theta, direction in rospy.get_param("/objectives/yellow/plant")]
        self.current_objective = self.objectives[0]

        while (self.path or self.objectives) and max_time > time.time() - self.start_time:
            self.close_enough_to_waypoint()
            self.compute_path()
            self.follow_path(self.current_objective.direction)

        rospy.loginfo("(STRATEGY) Plant phase is over")

    def solar_phase(self):
        rospy.loginfo("(STRATEGY) Starting solar phase")
        max_time = rospy.get_param("/phases/solar_panel")

        if self.team == TEAM_BLUE:
            solar_objectives = [
                Objective(x, y, theta, sqrt((x - self.position.x) ** 2 + (y - self.position.y) ** 2),
                          direction) for
                x, y, theta, direction in rospy.get_param("/objectives/blue/solar_panel")]
        else:
            solar_objectives = [
                Objective(x, y, theta, sqrt((x - self.position.x) ** 2 + (y - self.position.y) ** 2),
                          direction) for
                x, y, theta, direction in rospy.get_param("/objectives/yellow/solar_panel")]

        for solar_objective in solar_objectives:
            # Move to start position
            self.objectives = [solar_objective]
            self.current_objective = self.objectives[0]
            need_twice = False

            rospy.loginfo(f"(STRATEGY) Moving to solar panel at {solar_objective}")

            self.go_to(solar_objective.x, solar_objective.y, -1, .25, BACKWARD)

            rospy.loginfo(f"(STRATEGY) Waiting for robot to be ready")
            self.wait_until_ready()
            # while (self.path or self.objectives) and max_time > time.time() - self.start_time:
            #     self.close_enough_to_waypoint()
            #     self.compute_path()
            #     self.follow_path(self.current_objective.direction)

            rospy.loginfo(f"(STRATEGY) Arrived at solar panel at {solar_objective}")

            # Rotate self
            self.rotate_only(3 * pi / 2)

            rospy.loginfo(f"(STRATEGY) Rotated to solar panel at 3pi/2")
            # Get arm in the right position
            self.need_solar_winner = True
            rospy.loginfo(f"(STRATEGY) Waiting for solar panel winner. Current winner : {self.latest_solar_winner}")
            while self.latest_solar_winner == SOLAR_DEFAULT:
                rospy.sleep(0.1)

            rospy.loginfo(f"(STRATEGY) Solar panel winner : {self.latest_solar_winner}")
            if self.team == TEAM_BLUE and self.latest_solar_winner == SOLAR_NEUTRAL or self.team == TEAM_YELLOW and self.latest_solar_winner == SOLAR_BOTH:
                self.solar_pub.publish(Int16(180))

            if self.team == TEAM_BLUE and self.latest_solar_winner == SOLAR_BLUE or self.team == TEAM_YELLOW and self.latest_solar_winner == SOLAR_YELLOW:
                continue

            if self.team == TEAM_BLUE and self.latest_solar_winner == SOLAR_YELLOW or self.team == TEAM_YELLOW and self.latest_solar_winner == SOLAR_BLUE:
                need_twice = True

            self.latest_solar_winner = SOLAR_DEFAULT

            rospy.loginfo(f"(STRATEGY) Solar panel mode set")
            self.solar_mode_pub.publish(True)
            rospy.sleep(.1)
            # Bump
            rospy.loginfo(f"(STRATEGY) back until bumper")
            self.back_until_bumper()

            # Forward
            rospy.loginfo(f"(STRATEGY) Forward")
            self.go_to(self.position.x, self.position.y - .05, 3*pi/2, .15, FORWARD, Y_MINUS)
            self.wait_until_ready()
            self.solar_mode_pub.publish(False)
            # Rotate solar panel
            self.solar_pub.publish(Int16(90))
            rospy.sleep(.1)

            # Backwards
            rospy.loginfo(f"(STRATEGY) Backwards")
            self.back_until_bumper()

            # Reset arm
            rospy.loginfo(f"(STRATEGY) Reset arm")
            self.solar_pub.publish(Int16(0))
            rospy.sleep(.1)

            if need_twice:
                rospy.loginfo(f"(STRATEGY) Needs second pass")
                # Forward
                self.go_to(self.position.x, self.position.y - .03, 3 * pi / 2, .15, FORWARD, Y_MINUS)

                # Rotate solar panel
                self.solar_pub.publish(Int16(90))
                rospy.sleep(.1)

                # Backwards
                self.back_until_bumper()

                # Reset arm
                self.solar_pub.publish(Int16(0))
                rospy.sleep(.1)

    def close_enough_to_waypoint(self, threshold=5.0):
        while self.path and sqrt((self.position.x - self.path[0].position[0]) ** 2 + (self.position.y - self.path[0].position[1]) ** 2) < threshold / self.resolution:
            self.path.pop(0)  # Remove if he is close enough to the current intermediate objective
        if sqrt((self.position.x - self.current_objective.x) ** 2 + (self.position.y - self.current_objective.y) ** 2) < threshold / self.resolution:
            self.current_objective = None

    def close_enough_raw_waypoint(self, threshold=10.0):
        while self.raw_path and sqrt((self.position.x - self.raw_path[0].position[0]) ** 2 + (self.position.y - self.raw_path[0].position[1]) ** 2) < threshold / self.resolution:
            self.raw_path.pop(0)


    def compute_path(self):
        """Aggregate all the data and compute the path to follow using A* algorithm. Neighbors are defined by a dict of
        the form {direction: (cost, neighbor_node)}. The cost is very high if the neighbor is an obstacle.
        :return: the path to follow
        """
        rospy.loginfo(f"Data : \nL: {self.lidar_data}, \nC: {self.enemy_position} \n SELF POS : {self.position}")
        self.close_enough_raw_waypoint()
        self.obstacles1, self.obstacles2 = get_discrete_obstacles(self.lidar_data, self.us_data,
                                                [(self.enemy_position.x, self.enemy_position.y)],
                                                self.resolution, self.radius, self.map_boundaries, self.position)
        # rospy.loginfo(f"Obstacles : {len(self.obstacles)}")
        self.maze = update_maze(self.maze, self.previous_obstacles, self.obstacles1)
        self.previous_obstacles = self.obstacles1

        if self.current_objective is None:  # Get new closest objective
            self.reset_position_from_camera()
            self.current_objective = self.objectives[0]
            self.objectives.pop(0)
            self.raw_path = []
            rospy.loginfo(f"(STRATEGY) New objective : {self.current_objective}")
            # rospy.loginfo(f"(STRATEGY) Remaining objectives : {self.objectives}")

        # Get the start and end nodes
        origin = self.maze[int(self.position.x * self.resolution)][int(self.position.y * self.resolution)]
        origin.orientation = self.position.theta

        # rospy.loginfo(f"(STRATEGY) Current start/end : {origin.position}/{self.current_objective}")
        if is_path_valid(self.raw_path, self.obstacles2):  # Check if the path is still valid with a 10 cm margin
            rospy.loginfo("(STRATEGY) Path still exists")
        else:  # Compute a new path
            rospy.loginfo(f"(STRATEGY) Computing path from {origin.position} to {self.current_objective}")
            # save variables using pickle
            self.game_states.append([origin.position,
                                     self.maze[int(self.current_objective.x * self.resolution)][
                                         int(self.current_objective.y * self.resolution)].position,
                                     self.path, self.obstacles1, self.obstacles2, self.resolution, self.map_boundaries])
            with open("all_game_states.pkl", "wb") as f:
                pickle.dump(self.game_states, f)
            self.raw_path = a_star(origin, self.maze[int(self.current_objective.x * self.resolution)]
                                                    [int(self.current_objective.y * self.resolution)])
            rospy.loginfo(f"(STRATEGY) new Raw path computed : {self.raw_path}")
            self.path = clean_path(self.raw_path)
            self.path = meters_to_units(self.path, self.resolution)
            # rospy.loginfo(f"(STRATEGY) Converted path : {self.path}")

        # Remove node if the robot is already on it if the robot is already following a path
        self.close_enough_to_waypoint(threshold=4.0)
        # save_game_state(self.maze, self.path, self.obstacles, self.resolution, self.map_boundaries, "maze.png")
        rospy.loginfo(f"(STRATEGY) Path : {self.path}")

    def go_to(self, x=-1, y=-1, alpha=-1., speed=0.30, direction=BEST_DIRECTION, on_axis=NO_AXIS_MODE):
        """go to position (x, y, alpha)
        -> if alpha = -1 go to (x,y)
        -> direction = [0 : best option, 1 : forward, -1 : backward]"""
        # rospy.loginfo(f"Robot at {self.position}")
        obj = Pose2D()
        obj.x = x
        obj.y = y
        obj.theta = alpha
        self.next_pos_obj = [x, y, alpha]
        direction_data = Int16()
        direction_data.data = direction
        speed_data = Float64()
        speed_data.data = speed
        self.axis_mode_pub.publish(Int16(on_axis))
        self.direction_pub.publish(direction_data)
        self.speed_ctrl_pub.publish(speed_data)
        self.pos_ordre_pub.publish(obj)
        self.state_robot = IN_PROGRESS

    def wait_until_ready(self):
        while self.state_robot != READY:
            # rospy.loginfo("(STRATEGY) Waiting for the robot to be ready...")
            self.custom_waiting_rate.sleep()

    def follow_path(self, direction=BEST_DIRECTION):
        if self.path:
            rospy.loginfo(f"(STRATEGY) Following path : {self.path}")
            self.go_to(self.path[0].position[0], self.path[0].position[1], -1, DEFAULT_MAX_SPEED, direction)
            rospy.loginfo(f"(STRATEGY) Going to {self.path[0]}")
        else:
            rospy.loginfo("(STRATEGY) No path found")

    def setup_subscribers(self):
        rospy.Subscriber('odometry', Pose2D, self.update_position)
        rospy.Subscriber('lidar_data', PoseArray, self.update_lidar_data)
        rospy.Subscriber('ultrasound_sensor_data', Float32MultiArray, self.update_us_data)
        rospy.Subscriber('camera', Float32MultiArray, self.update_camera)
        rospy.Subscriber('solar_aruco', Int8, self.update_solar_winner)
        rospy.Subscriber('bumper', Byte, self.update_bumpers)
        rospy.Subscriber('state', Int16, self.update_state)
        rospy.Subscriber('Team', Bool, self.update_team)

    def reset_position_from_camera(self, wait : float = .3):
        """Publishes the camera position to the odometry topic to correct the odometry"""
        self.wait_until_ready()
        rospy.sleep(wait)
        if time.time() - self.last_time_cam < 2:
            self.got_cam_data = False
            while not self.got_cam_data:
                rospy.sleep(0.05)
            if self.camera_position.x != -1 and self.camera_position.y != -1:
                self.publisher_correct_odom.publish(self.camera_position)
        else:
            rospy.logwarn("(STRATEGY) No connexion with camera")
            return False
        self.need_rst_odom = False
        self.got_cam_data = False
        self.position = self.camera_position
        return True

    def update_bumpers(self, data: Byte):
        """Update the bumper states by reading the Byte message from the bumper topic."""
        data = int(data.data)
        for i in range(4):
            if data & 2 ** i:
                setattr(self, f'bumper_{i + 1}', True)
            else:
                setattr(self, f'bumper_{i + 1}', False)

            rospy.loginfo(f"(STRATEGY) Bumper {i + 1} : {getattr(self, f'bumper_{i + 1}')}")
        self.need_for_compute = True

    def update_position(self, data):
        self.position = data
        # rospy.loginfo(f"(STRATEGY) Position received : {self.position}")
        self.need_for_compute = True

    def update_lidar_data(self, data: PoseArray):
        raw = data
        self.lidar_data = [(raw.poses[i].position.x, raw.poses[i].position.y) for i in range(len(raw.poses))]
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
        if self.team == TEAM_BLUE:
            blue_position, yellow_position = blue_robot, yellow_robot
        else:
            blue_position, yellow_position = yellow_robot, blue_robot
        self.camera_position, self.enemy_position = parse_camera_data(yellow_robot, blue_robot, blue_position, yellow_position)
        self.camera_position.theta = clamp_theta(self.camera_position.theta)
        self.enemy_position.theta = clamp_theta(self.enemy_position.theta)

        self.got_cam_data = True
        self.last_time_cam = time.time()

    def update_team(self, data: Bool):
        global start
        if not start or self.team != -1:
            self.team = TEAM_BLUE if data.data else TEAM_YELLOW
        else:
            rospy.loginfo("(STRATEGY) YOU CAN'T CHANGE TEAM AFTER STARTING THE GAME !")

    def update_state(self, data: Int16):
        self.state_robot = data.data
        self.need_for_compute = True

    def update_solar_winner(self, winner: Int8):
        if self.need_solar_winner:
            self.need_solar_winner = False
            self.latest_solar_winner = winner.data
    def stop(self):
        self.go_to(self.position.x, self.position.y)  # Stop the robot

    def back_until_bumper(self, speed=0.15, axis='y+', direction=BACKWARD):
        while not self.bumper_1 and not self.bumper_2 and not self.bumper_3 and not self.bumper_4:
            if axis == 'y+':
                self.go_to(self.position.x, self.position.y + 2, speed=speed, direction=direction, on_axis=Y_PLUS)
            elif axis == 'y-':
                self.go_to(self.position.x, self.position.y - 2, speed=speed, direction=direction, on_axis=Y_MINUS)
            elif axis == 'x+':
                self.go_to(self.position.x + 3, self.position.y, speed=speed, direction=direction, on_axis=X_PLUS)
            elif axis == 'x-':
                self.go_to(self.position.x - 3, self.position.y, speed=speed, direction=direction, on_axis=X_MINUS)
        self.stop()

    def move_relative(self, distance, speed=0.20, axis='y-', direction=BEST_DIRECTION):
        if axis == 'y-':
            self.go_to(self.position.x, self.position.y - distance, speed=speed, direction=direction, on_axis=Y_MINUS)
        elif axis == 'y+':
            self.go_to(self.position.x, self.position.y + distance, speed=speed, direction=direction, on_axis=Y_PLUS)
        elif axis == 'x-':
            self.go_to(self.position.x - distance, self.position.y, speed=speed, direction=direction, on_axis=X_MINUS)
        elif axis == 'x+':
            self.go_to(self.position.x + distance, self.position.y, speed=speed, direction=direction, on_axis=X_PLUS)
        self.wait_until_ready()

    def rotate_only(self, angle : float, speed : float = .2):
        self.go_to(self.position.x, self.position.y, angle, speed, BEST_DIRECTION)
        self.reset_position_from_camera()
        self.wait_until_ready()
        while abs(self.camera_position.theta - angle) > 0.05:                                      ####### verifier cette ligne entre position depuis odom ou position depuis cammmera
            rospy.loginfo(f"(STRATEGY) Correcting angle : {self.position.theta} -> {angle}")
            self.go_to(self.position.x, self.position.y, angle, speed, BEST_DIRECTION)
            self.reset_position_from_camera(.1)


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

        if not strategy_manager.reset_position_from_camera():
            rospy.logwarn("Failed to reset the position from the camera. Applying default position.")
            if strategy_manager.team == TEAM_BLUE:
                strategy_manager.position = Pose2D(.2, 1., 0.0)
                strategy_manager.publisher_correct_odom.publish(Pose2D(.2, 1., 0.0))
            else:
                strategy_manager.position = Pose2D(2.8, 1., pi)
                strategy_manager.publisher_correct_odom.publish(Pose2D(2.8, 1., pi))

        rospy.sleep(0.1)
        strategy_manager.run()
    finally:
        rospy.loginfo("[STOP] Strategy node has stopped.")
