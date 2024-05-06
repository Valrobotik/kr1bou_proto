#!/usr/bin/env python3

import time
from math import sqrt

import rospy
from geometry_msgs.msg import Pose2D, PoseArray
from std_msgs.msg import Float64, Bool, Int8, Int16, Float32MultiArray, Byte

from search_path import a_star, clean_path
from utils import *

import pickle
import cProfile
import pstats
import math
import io

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

DEFAULT_MAX_SPEED = 0.32

NO_AXIS_MODE = 0
X_PLUS = 1
X_MINUS = 2
Y_PLUS = 3
Y_MINUS = 4
PREFERRED_AXIS = 5

SAVE_GAME_STATE = False


class Strategy:
    def __init__(self) -> None:
        # -- Robot related --
        self.need_for_compute = True  # Whether to ask for a new path
        self.next_pos_obj = [0, 0, 0]  # Next position to go to / Intermediate objective
        self.game_states = []
        self.current_max_time = 0

        # -- Map/Graph related --
        self.map_boundaries = [int(m) for m in rospy.get_param('/map_boundaries')]
        self.resolution = rospy.get_param('/resolution')  # Resolution to centimeters for example.
        self.radius = rospy.get_param('/radius')  # Radius of the robot in the resolution/unit given.
        self.raw_path = []  # Raw path to follow
        self.path = []  # List of waypoints to follow
        self.obstacles1 = set()  # List of obstacles with radius
        self.obstacles2 = set()  # List of obstacles with radius - 10
        self.previous_obstacles = set()  # Previous obstacles
        self.maze_shape = (int(self.map_boundaries[2] * self.resolution), int(self.map_boundaries[3] * self.resolution))
        self.maze = np.zeros(self.maze_shape, dtype=Node)
        self.maze = setup_maze(self.maze, self.obstacles1)
        self.custom_waiting_rate = rospy.Rate(20)

        # -- Variables for Subscribers --
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
        self.bumpers = 0
        # State of the robot
        self.state_robot = READY
        # Team Color
        self.team = -1
        # -- Subscribers --
        self.setup_subscribers()

        # -- Strategy related --
        self.objectives = []
        self.current_objective = None
        self.current_direction = BEST_DIRECTION
        self.start_time = float("inf")

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

        # Start debug
        pr.enable()
        self.start_time = time.time()
        # self.debug_phase()
        # self.debug_phase_goto()
        # self.debug_phase_rotate()
        self.plant_phase()
        # self.solar_phase()
        # self.home_phase()
        rospy.loginfo("(STRATEGY) Strategy running loop has stopped.")

    def update_current_objective(self):
        if self.current_objective is None:  # Get new closest objective
            self.reset_position_from_camera()
            self.current_objective = self.objectives[0]
            self.objectives.pop(0)
            self.raw_path = []
            rospy.loginfo(f"(STRATEGY) New objective : {self.current_objective}")

    def compute_path(self):
        """Aggregate all the data and compute the path to follow using A* algorithm. Neighbors are defined by a dict of
        the form {direction: (cost, neighbor_node)}. The cost is very high if the neighbor is an obstacle.
        :return: the path to follow
        """
        rospy.loginfo(f"Data : \nL: {self.lidar_data}, \nC: {self.enemy_position} \n SELF POS : {self.position}")
        self.obstacles1, self.obstacles2 = get_discrete_obstacles(self.lidar_data, self.us_data,
                                                                  [(self.enemy_position.x, self.enemy_position.y)],
                                                                  self.resolution, self.radius, self.map_boundaries,
                                                                  self.position)

        if is_path_valid(self.raw_path, self.obstacles2):  # Check if the path is still valid with a 10 cm margin
            rospy.loginfo("(STRATEGY) Path still exists")
        else:  # Compute a new path
            self.maze = update_maze(self.maze, self.previous_obstacles, self.obstacles1)
            self.previous_obstacles = self.obstacles1

            # Get the start and end nodes
            origin = self.maze[int(self.position.x * self.resolution)][int(self.position.y * self.resolution)]
            origin.orientation = self.position.theta
            end = self.maze[int(self.current_objective.x * self.resolution)][int(self.current_objective.y * self.resolution)]
            end.orientation = self.current_objective.theta
            rospy.loginfo(f"(STRATEGY) Computing path from {origin.position} to {end.position}")

            self.register_game_state(origin, end)

            # Before computing A*, check if a direct line is valid.
            straight_path = [self.maze[i][j] for i, j in bresenham(origin.position, end.position)]
            if is_path_valid(straight_path, self.obstacles2):
                self.raw_path = straight_path
                self.path = [straight_path[-1]]
                rospy.loginfo(f"(STRATEGY) Computed path using Bresenham : {self.raw_path}")
            else:
                self.raw_path = a_star(origin, end)
                rospy.loginfo(f"(STRATEGY) Computed path using A* : {self.raw_path}")
                self.path = clean_path(self.raw_path)
            self.path = meters_to_units(self.path, self.resolution)

    def follow_path(self, direction=BEST_DIRECTION):
        if self.path:
            rospy.loginfo(f"(STRATEGY) Following path : {self.path}")
            self.go_to(self.path[0].position[0], self.path[0].position[1], -1, DEFAULT_MAX_SPEED, direction)
            rospy.loginfo(f"(STRATEGY) Going to {self.path[0]} with direction {direction}")
        else:
            rospy.loginfo("(STRATEGY) No path to follow")

    def go_to(self, x=-1., y=-1., alpha=-1., speed=0.30, direction=BEST_DIRECTION, on_axis=NO_AXIS_MODE):
        """Update topic for going to a position (x, y, alpha) ==> Motion Control
        -> if alpha = -1 go to (x,y)
        -> direction = [0 : best option, 1 : forward, -1 : backward]
        """
        self.axis_mode_pub.publish(Int16(on_axis))
        self.direction_pub.publish(Int16(direction))
        self.speed_ctrl_pub.publish(Float64(speed))
        self.pos_ordre_pub.publish(Pose2D(x, y, alpha))
        self.state_robot = IN_PROGRESS

    # -- Phases --
    def debug_phase(self):
        rospy.loginfo("(STRATEGY) Starting debug phase")
        max_time = rospy.get_param("/phases/debug")
        self.objectives = self.parse_objectives("debug")

        while (self.path or self.objectives or self.current_objective) and max_time > time.time() - self.start_time:
            self.update_current_objective()
            self.close_enough_raw_waypoint()
            self.compute_path()
            self.close_enough_to_waypoint(threshold=4.0)  # remove close enough waypoints
            self.follow_path()

        rospy.loginfo("(STRATEGY) Debug phase is over")
        if max_time > time.time() - self.start_time:
            rospy.loginfo("Debug phase is over because of time")

    def debug_phase_goto(self):
        while not rospy.is_shutdown():
            rospy.loginfo("(STRATEGY) Going to 2.25, .5")
            self.go_to(2.25, .5, -1, .25, BEST_DIRECTION)
            self.wait_until_ready()
            rospy.loginfo("(STRATEGY) Going to 2.25, 1.5")
            self.go_to(2.25, 1.5, -1, .25, BEST_DIRECTION)
            self.wait_until_ready()
            rospy.loginfo("(STRATEGY) Going to .75, 1.5")
            self.go_to(.75, 1.5, -1, .25, BEST_DIRECTION)
            self.wait_until_ready()
            rospy.loginfo("(STRATEGY) Going to .75, .5")
            self.go_to(.75, .5, -1, .25, BEST_DIRECTION)
            self.wait_until_ready()

    def debug_phase_rotate(self):
        while not rospy.is_shutdown():
            rospy.loginfo("(STRATEGY) Rotating to 0")
            self.rotate_only(pi)
            self.wait_until_ready()

    def plant_phase(self):
        rospy.loginfo("(STRATEGY) Starting plant phase")
        times = list(rospy.get_param("/phases/plant").values())
        sequences = self.parse_sequences("plant")

        while sequences:
            self.objectives = sequences.pop(0)
            max_time = times.pop(0)
            max_sequence_time = rospy.get_param("/phases/plant")
            while (self.path or self.objectives or self.current_objective) and max_time > time.time() - self.start_time:
                if max_sequence_time < time.time() - self.start_time:
                    break
                self.update_current_objective()
                self.close_enough_raw_waypoint()
                self.compute_path()
                self.close_enough_to_waypoint(threshold=4.0)  # remove close enough waypoints
                self.follow_path(self.current_objective.direction)
            self.collect_paths()

        rospy.loginfo("(STRATEGY) Plant phase is over" + (": time over" if not sequences else ": next sequence"))

    def solar_alt_phase(self):
        rospy.loginfo("(STRATEGY) Starting solar alternative phase")
        self.current_max_time = rospy.get_param("/phases/solar_panel_alt")
        solar_objectives = self.parse_objectives("solar_panel_alt")

        for solar_objective in solar_objectives:
            self.objectives = [solar_objective]
            self.current_objective = self.objectives[0]
            startup_arm_pos = 0

            rospy.loginfo(f"(STRATEGY) going to original solar_pannel at {solar_objective}")

            while (self.path or self.objectives or self.current_objective) and self.phase_end():
                self.update_current_objective()
                
                self.compute_path()
                self.follow_path(self.current_objective.direction)
                self.close_enough_to_waypoint()
            rospy.loginfo(f"(STRATEGY) Arrived at solar panel at {solar_objective}")

            self.reset_position_from_camera()

            # Rotate self
            self.go_to(1, solar_objective.y, speed=0.2, direction=BACKWARD, on_axis=X_PLUS)
            self.reset_position_from_camera()
            if self.phase_end(): break
            self.go_to(2, solar_objective.y, 0, speed=0.2, direction=FORWARD, on_axis=X_PLUS)
            self.reset_position_from_camera()
            if self.phase_end(): break
        

    def phase_end(self):
        return self.current_max_time > time.time() - self.start_time




    def solar_phase(self):
        rospy.loginfo("(STRATEGY) Starting solar phase")
        max_time = rospy.get_param("/phases/solar_panel")
        solar_objectives = self.parse_objectives("solar_panel")

        for solar_objective in solar_objectives:
            # Move to start position
            self.objectives = [solar_objective]
            self.current_objective = self.objectives[0]
            need_twice = False
            startup_arm_pos = 0

            rospy.loginfo(f"(STRATEGY) Moving to solar panel at {solar_objective}")
            rospy.loginfo(f"(STRATEGY) Waiting for robot to be ready")
            self.wait_until_ready()

            while (self.path or self.objectives or self.current_objective) and max_time > time.time() - self.start_time:
                self.update_current_objective()
                self.compute_path()
                self.follow_path(self.current_objective.direction)
                self.close_enough_to_waypoint()
            rospy.loginfo(f"(STRATEGY) Arrived at solar panel at {solar_objective}")

            # Backwards
            self.reset_position_from_camera()
            self.go_to(solar_objective.x, solar_objective.y + .2, 3 * pi / 2, .2, BACKWARD, Y_PLUS)

            # Rotate self
            self.rotate_only(3 * pi / 2)
            rospy.loginfo(f"(STRATEGY) Rotated to solar panel at 3pi/2")
            self.need_solar_winner = True  # Get arm in the right position


            # Disable back bumpers and enable back camera
            self.latest_solar_winner = SOLAR_DEFAULT
            rospy.loginfo(f"(STRATEGY) Solar panel mode set")
            self.solar_mode_pub.publish(Bool(True))

            # Wait for solar panel winner
            rospy.loginfo(f"(STRATEGY) Waiting for solar panel winner. Current winner : {self.latest_solar_winner}")
            while self.latest_solar_winner == SOLAR_DEFAULT:
                rospy.sleep(0.1)

            # Check for winner
            rospy.loginfo(f"(STRATEGY) Solar panel winner : {self.latest_solar_winner}")
            if self.team == TEAM_BLUE and self.latest_solar_winner == SOLAR_NEUTRAL or self.team == TEAM_YELLOW and self.latest_solar_winner == SOLAR_BOTH:
                startup_arm_pos = 180

            if self.team == TEAM_BLUE and self.latest_solar_winner == SOLAR_BLUE or self.team == TEAM_YELLOW and self.latest_solar_winner == SOLAR_YELLOW:
                continue

            if self.team == TEAM_BLUE and self.latest_solar_winner == SOLAR_YELLOW or self.team == TEAM_YELLOW and self.latest_solar_winner == SOLAR_BLUE:
                need_twice = True

            self.solar_pub.publish(Int16(startup_arm_pos))

            # Bump
            rospy.loginfo(f"(STRATEGY) back until bumper")
            self.back_until_bumper()

            # Forward
            rospy.loginfo(f"(STRATEGY) Forward")
            #calcule de la position du robot avancé
            obj_x = self.position.x + 0.03 * math.cos(self.position.theta)
            obj_y = self.position.y + 0.03 * math.sin(self.position.theta)
            self.go_to(obj_x, obj_y, -1, .15, FORWARD, PREFERRED_AXIS)
            self.wait_until_ready()
            # Rotate solar panel
            rospy.loginfo(f"(STRATEGY) Rotate solar panel")
            self.solar_pub.publish(Int16(90))
            rospy.sleep(.5)

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
                # Reset arm
                self.solar_pub.publish(Int16(0))
                rospy.sleep(.1)

            # Forward
            self.solar_mode_pub.publish(Bool(False))
            self.go_to(self.position.x, self.position.y - .2, 3 * pi / 2, .15, FORWARD, Y_PLUS)

    # -- Utils --
    def parse_sequences(self, phase):
        team = "blue" if self.team == TEAM_BLUE else "yellow"
        return [self.parse_objectives(team, phase, i) for i in range(len(rospy.get_param(f"/objectives/{team}/{phase}").values()))]

    def parse_objectives(self, team, phase, index_of_sequence):
        return [Objective(x, y, theta, direction) for
                x, y, theta, direction in rospy.get_param(f"/objectives/{team}/{phase}/sequence{index_of_sequence}")]

    def close_enough_to_waypoint(self, threshold=5.0):
        while self.path and sqrt((self.position.x - self.path[0].position[0]) ** 2 + (
                self.position.y - self.path[0].position[1]) ** 2) < threshold / self.resolution:
            self.path.pop(0)  # Remove if he is close enough to the current intermediate objective
        if sqrt((self.position.x - self.current_objective.x) ** 2 + (
                self.position.y - self.current_objective.y) ** 2) < threshold / self.resolution:
            self.current_objective = None

    def close_enough_raw_waypoint(self, threshold=10.0):
        while self.raw_path and sqrt((self.position.x - self.raw_path[0].position[0]) ** 2 + (
                self.position.y - self.raw_path[0].position[1]) ** 2) < threshold:
            self.raw_path.pop(0)

    def collect_paths(self):
        self.raw_path = []
        self.path = []
        self.current_objective = None

    def register_game_state(self, origin, end):
        if SAVE_GAME_STATE:
            self.game_states.append([origin.position, end.position,
                                     self.path, self.obstacles1, self.obstacles2, self.resolution, self.map_boundaries])
            with open("all_game_states.pkl", "wb") as f:
                pickle.dump(self.game_states, f)

    def wait_until_ready(self):
        while self.state_robot != READY:
            # rospy.loginfo("(STRATEGY) Waiting for the robot to be ready...")
            self.custom_waiting_rate.sleep()
            if self.phase_end(): return

    def reset_position_from_camera(self, wait: float = .3):
        """Publishes the camera position to the odometry topic to correct the odometry"""
        self.wait_until_ready()
        if self.phase_end(): return
        rospy.sleep(wait)
        if time.time() - self.last_time_cam < 2:
            self.got_cam_data = False
            while not self.got_cam_data:
                rospy.sleep(0.05)
            self.publisher_correct_odom.publish(self.camera_position)
        else:
            rospy.logwarn("(STRATEGY) No connexion with camera")
            return False
        self.need_rst_odom = False
        self.got_cam_data = False
        self.position = self.camera_position
        return True

    def stop(self):
        self.go_to(self.position.x, self.position.y)  # Stop the robot

    def is_activated_bumper(self, ids):
        """Return True if all ids of bumpers are activated."""
        for i in ids:
            if self.bumpers & (1 << i):
                return True
        return False

    def back_until_bumper(self, speed : float = 0.2, axis : str = 'y+', direction : int = BACKWARD, shift : int = 10):
        # shift /= self.resolution
        while not self.is_activated_bumper([2, 3]):  # back bumpers should be activated
            rospy.loginfo(f"Back bumpers not activated: {self.bumpers}")

            if axis == 'y+':
                self.go_to(self.position.x + .1, self.position.y + shift, speed=speed, direction=direction, on_axis=Y_PLUS)
            elif axis == 'y-':
                self.go_to(self.position.x - .1, self.position.y - shift, direction=direction, on_axis=Y_MINUS)
            elif axis == 'x+':
                self.go_to(self.position.x + shift, self.position.y, speed=speed, direction=direction, on_axis=X_PLUS)
            elif axis == 'x-':
                self.go_to(self.position.x - shift, self.position.y, speed=speed, direction=direction, on_axis=X_MINUS)
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

    def rotate_only(self, angle: float):
        self.go_to(-1, -1, angle)

        use_cam = self.reset_position_from_camera()

        while (abs(self.camera_position.theta - angle) > 0.15 and use_cam) or (abs(self.position.theta - angle) > 0.15 and not use_cam):
            rospy.loginfo(f"(STRATEGY) Correcting angle : {self.position.theta} -> {angle}")
            self.go_to(-1, -1, angle)
            use_cam = self.reset_position_from_camera()
        rospy.loginfo(f"(STRATEGY) Rotated to {angle} SUCESSFULLY")

    def setup_subscribers(self):
        rospy.Subscriber('odometry', Pose2D, self.update_position)
        rospy.Subscriber('lidar_data', PoseArray, self.update_lidar_data)
        rospy.Subscriber('ultrasound_sensor_data', Float32MultiArray, self.update_us_data)
        rospy.Subscriber('camera', Float32MultiArray, self.update_camera)
        rospy.Subscriber('solar_aruco', Int8, self.update_solar_winner)
        rospy.Subscriber('bumper', Byte, self.update_bumpers)
        rospy.Subscriber('state', Int16, self.update_state)
        rospy.Subscriber('team', Bool, self.update_team)

    # -- Callbacks --
    def update_bumpers(self, data: Byte):
        """Update the bumper states by reading the Byte message from the bumper topic."""
        self.bumpers = int(data.data)
        self.need_for_compute = True

    def update_position(self, data):
        self.position = data
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
            own_robot, enemy_robot = blue_robot, yellow_robot
        else:
            own_robot, enemy_robot = yellow_robot, blue_robot
        camera_position, enemy_position = parse_camera_data(own_robot, enemy_robot)
        if camera_position is not None:
            self.camera_position = camera_position
            self.camera_position.theta = clamp_theta(self.camera_position.theta)
        if enemy_position is not None:
            self.enemy_position = enemy_position
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


def run(data):
    global start
    start = data
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from RunningPhase")


if __name__ == "__main__":
    # profile le code stratégie
    pr = cProfile.Profile()
    try:
        start = False
        rospy.init_node("strategy")
        rospy.loginfo("[START] Strategy node has started.")
        strategy_manager = Strategy()

        rospy.Subscriber('running_phase', Bool, run)
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
        pr.disable()
        s = io.StringIO()
        ps = pstats.Stats(pr, stream=s).sort_stats("cumulative")
        # Sauvegarde des résultats dans un fichier
        ps.print_stats()

        with open('cProfile.txt', 'w') as f:
            f.write(s.getvalue())