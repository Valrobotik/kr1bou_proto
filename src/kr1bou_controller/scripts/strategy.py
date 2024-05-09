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

SLOW_SPEED = 0.15
MEDIUM_SPEED = 0.25
MAX_SPEED = 0.32

NO_AXIS_MODE = 0
X_PLUS = 1
X_MINUS = 2
Y_PLUS = 3
Y_MINUS = 4
PREFERRED_AXIS = 5

SAVE_GAME_STATE = False
PROFILE = False


class Strategy:
    def __init__(self) -> None:
        # -- Robot related --  # Whether to ask for a new path
        self.next_pos_obj = [0, 0, 0]  # Next position to go to / Intermediate objective
        self.game_states = []
        self.current_max_time = float("inf")
        self.points_counter = 0
        self.custom_waiting_rate = rospy.Rate(20)

        # -- Map --
        self.map_boundaries = [int(m) for m in rospy.get_param('/map_boundaries')]
        self.resolution = rospy.get_param('/resolution')  # Resolution to centimeters for example.
        self.radius = rospy.get_param('/radius')  # Radius of the robot in the resolution/unit given.

        # -- Variables for Subscribers --
        self.position = Pose2D()  # Get the initial position of the robot
        self.enemy_position = Pose2D()  # Get the position of the enemy robot
        self.lidar_data = []  # Get the lidar data
        self.us_data = [(-1, -1) for _ in range(10)]
        self.latest_solar_winner = SOLAR_DEFAULT
        self.need_solar_winner = False
        self.need_rst_odom = False
        self.last_time_cam = time.time()
        self.camera_position = Pose2D()
        self.got_cam_data = False
        # Arm
        self.arm_angle = 0
        # Bumpers
        self.bumpers = 0
        self.state_robot = READY
        self.team = -1

        # -- Subscribers --
        self.setup_subscribers()

        # -- Strategy related --
        self.start_time = float("inf")
        self.objectives = []
        self.current_objective = None
        self.path, self.raw_path = [], []
        self.large_obstacles, self.thin_obstacles, self.previous_obstacles = set(), set(), set()
        self.maze_shape = (int(self.map_boundaries[2] * self.resolution), int(self.map_boundaries[3] * self.resolution))
        self.maze = setup_maze(np.zeros(self.maze_shape, dtype=Node),
                               setup_map_boundaries_obstacles(self.map_boundaries, pami_obstacles=rospy.get_param('/pami_obstacles')))

        # -- Publishers --
        self.solar_pub = rospy.Publisher('solar_angle', Int16, queue_size=1)
        self.pos_ordre_pub = rospy.Publisher('next_objectif', Pose2D, queue_size=1)
        self.direction_pub = rospy.Publisher('direction', Int16, queue_size=1)
        self.speed_ctrl_pub = rospy.Publisher('max_speed', Float64, queue_size=1)
        self.publisher_correct_odom = rospy.Publisher('odom_corrected', Pose2D, queue_size=1)
        self.solar_mode_pub = rospy.Publisher('solar_mode', Bool, queue_size=1)
        self.axis_mode_pub = rospy.Publisher('axis_mode', Int16, queue_size=1)
        self.points_pub = rospy.Publisher('points', Int8, queue_size=1)

    def run(self):
        rospy.loginfo("(STRATEGY) Strategy running loop has started.")
        while self.team == -1 and not rospy.is_shutdown():
            rospy.sleep(0.05)

        pr.enable() if PROFILE else None
        # Move arm
        self.arm_angle = 180 if self.team == TEAM_BLUE else 0
        self.solar_pub.publish(Int16(self.arm_angle))
        self.add_points(12) # Publish first points
        self.start_time = time.time()
        
        # Phases
        # self.debug_phase_goto() # self.debug_phase_rotate()
        self.plant_phase()
        self.solar_phase()
        self.home_phase()
        rospy.loginfo("(STRATEGY) Strategy running loop has stopped.")

    # -- Phases --
    def home_phase(self):
        rospy.loginfo("(STRATEGY) Starting home phase")
        times = list(rospy.get_param("/phases/home").values())
        points = list(rospy.get_param("/points/home").values())
        sequences = self.parse_sequences("home")

        self.follow_best_sequence(sequences, times, points)
        rospy.loginfo("(STRATEGY) Home phase is over" + (": time over" if not sequences else ": next sequence"))

    def solar_phase(self):
        rospy.loginfo("(STRATEGY) Starting solar phase")
        times = list(rospy.get_param("/phases/solar_panel").values())
        points = list(rospy.get_param("/points/solar_panel").values())

        self.follow_sequences(sequences := self.parse_sequences("solar_panel"), times, points)  # Do phase

        # Move arm back
        self.solar_pub.publish(Int16(abs(self.arm_angle - 180)))
        rospy.loginfo("(STRATEGY) Solar phase is over" + (": time over" if not sequences else ": next sequence"))

    def plant_phase(self):
        rospy.loginfo("(STRATEGY) Starting plant phase")
        times = list(rospy.get_param("/phases/plant").values())
        points = list(rospy.get_param("/points/plant").values())

        alternative = False  # whether to get to the default of alternative phase
        sequences = self.parse_sequences("plant", alternative)
        self.follow_sequences(sequences, times, points)
        rospy.loginfo("(STRATEGY) Plant phase is over" + (": time over" if not sequences else ": next sequence"))

    # -- Core functions --
    def follow_sequences(self, sequences, times, points):
        while sequences:
            self.set_objectives(sequences.pop(0))
            self.follow_sequence(max_time := times.pop(0), BEST_DIRECTION, MAX_SPEED)

            if time.time() < max_time or self.current_objective is None:
                self.add_points(points.pop(0))
            self.collect_paths()

    def follow_sequence(self, max_time, direction, speed):
        while (self.path or self.objectives or self.current_objective) and max_time > time.time() - self.start_time:
            self.update_current_objective()
            direction = self.current_objective.direction if self.current_objective else direction
            speed = self.current_objective.speed if self.current_objective else speed
            self.compute_path()
            self.close_enough_raw_waypoint()
            self.close_enough_to_waypoint(threshold=4.0)  # remove close enough waypoints
            self.follow_path(speed, direction)

    def follow_best_sequence(self, sequences, times, points):
        """We assume that the sequences are sorted by priority."""
        rospy.loginfo(f"(STRATEGY) Points : {points}")
        rospy.loginfo(f"(STRATEGY) Times: {times}")
        chosen_sequence = None
        max_time = times[0]
        seq_points = points[0]

        while not chosen_sequence:
            for sequence in sequences:
                self.set_objectives(sequence)
                self.update_current_objective()
                self.compute_path()
                self.close_enough_raw_waypoint()
                self.close_enough_to_waypoint(threshold=4.0)
                if self.path:
                    rospy.loginfo(f"(STRATEGY) Sequence found : {sequence}")
                    chosen_sequence = sequence
                    self.set_objectives(sequence)
                    break
                rospy.sleep(.1)

        if not chosen_sequence:
            rospy.logwarn("(STRATEGY) No sequence found. Using first sequence as fallback.")
            self.set_objectives(sequences[0])

        self.follow_sequence(max_time, BEST_DIRECTION, MAX_SPEED)
        if time.time() < max_time or self.current_objective is None:
            self.add_points(seq_points)
        self.collect_paths()

    def update_current_objective(self):
        if self.current_objective is None:
            self.reset_position_from_camera()
            if self.objectives:
                self.current_objective = self.get_objective(0, pop=True)
            self.set_raw_path([])
            rospy.loginfo(f"(STRATEGY) New objective : {self.current_objective}")

    def compute_path(self):
        """Compute a new path for a given objective if necessary. The path is obtained using Bresenham or A*."""
        rospy.loginfo(f"\nData : \n\tL: {self.lidar_data}, \n\tC: {self.enemy_position} \n\tS: {self.position}")
        self.large_obstacles, self.thin_obstacles = get_discrete_obstacles(self.lidar_data, self.us_data,
                                                                           [(self.enemy_position.x, self.enemy_position.y)],
                                                                           self.resolution, self.radius, self.map_boundaries,
                                                                           self.position)  # Update obstacles and tolerance zone

        # Check if the path is still valid (with a certain tolerance)
        if is_path_valid(self.raw_path, self.thin_obstacles):
            rospy.loginfo("(STRATEGY) Path still exists")
        else:  # Compute a new path
            self.maze = update_maze(self.maze, self.previous_obstacles, self.large_obstacles)
            self.previous_obstacles = self.large_obstacles

            # Get the start and end nodes
            origin = self.maze[int(self.position.x * self.resolution)][int(self.position.y * self.resolution)]
            origin.orientation = self.position.theta
            end = self.maze[int(self.current_objective.x * self.resolution)][int(self.current_objective.y * self.resolution)]
            end.orientation = self.current_objective.theta
            #rospy.loginfo(f"(STRATEGY) Computing path from {origin} to {end}")

            self.register_game_state(origin, end) if SAVE_GAME_STATE else None

            # Before computing A*, check if a direct line is valid.
            straight_path = [self.maze[i][j] for i, j in bresenham(origin, end)]
            if is_path_valid(straight_path, self.thin_obstacles):
                self.set_raw_path(straight_path)
                self.set_path([straight_path[-1]])
                #rospy.loginfo(f"(STRATEGY) Computed path using Bresenham : {self.raw_path}")
            else:
                self.set_raw_path(a_star(origin, end))
                #rospy.loginfo(f"(STRATEGY) Computed path using A* : {self.raw_path}")
                self.set_path(clean_path(self.raw_path))
            self.set_path(meters_to_units(self.path, self.resolution))

    def follow_path(self, speed=MAX_SPEED, direction=BEST_DIRECTION):
        if self.path:
            #ospy.loginfo(f"(STRATEGY) Following path : {self.path}")
            self.go_to(self.get_path(0).x, self.get_path(0).y, -1, speed, direction)
            #rospy.loginfo(f"(STRATEGY) Going to {self.get_path(0)} with direction {direction}")
        else:
            rospy.loginfo("(STRATEGY) No path to follow")

    def go_to(self, x=-1., y=-1., alpha=-1., speed=0.30, direction=BEST_DIRECTION, on_axis=NO_AXIS_MODE):
        self.axis_mode_pub.publish(Int16(on_axis))
        self.direction_pub.publish(Int16(direction))
        self.speed_ctrl_pub.publish(Float64(speed))
        self.pos_ordre_pub.publish(Pose2D(x, y, alpha))
        self.state_robot = IN_PROGRESS

    # -- Utils --
    def add_points(self, points):
        self.points_counter += points
        self.points_pub.publish(Int8(self.points_counter))
        rospy.loginfo(f"(STRATEGY) Added points. Score: {self.points_counter}")

    def phase_end(self):
        return self.current_max_time < time.time() - self.start_time

    def parse_sequences(self, phase, alternative=False):
        if alternative:
            return [self.parse_objectives(phase, i) for i in range(len(rospy.get_param(f"/objectives/{phase}_alternative").values()))]
        return [self.parse_objectives(phase, i) for i in range(len(rospy.get_param(f"/objectives/{phase}").values()))]

    def parse_objectives(self, phase, index_of_sequence):
        return [Objective(x, y, theta, speed, direction, self.team) for
                x, y, theta, speed, direction in rospy.get_param(f"/objectives/{phase}/sequence{index_of_sequence}")]

    def close_enough_to_waypoint(self, threshold=5.0):
        while self.path and sqrt((self.position.x - self.get_path(0).x) ** 2 + (
                self.position.y - self.get_path(0).y) ** 2) < threshold / self.resolution:
            self.get_path(0, pop=True)  # Remove if he is close enough to the current intermediate objective
        if sqrt((self.position.x - self.current_objective.x) ** 2 + (
                self.position.y - self.current_objective.y) ** 2) < threshold / self.resolution:
            self.collect_paths()

    def close_enough_raw_waypoint(self, threshold=10.0):
        if not self.raw_path:
            return
        last_node_to_remove = None
        for node in self.raw_path:
            if sqrt((self.position.x - node.x) ** 2 + (self.position.y - node.y) ** 2) < threshold:
                last_node_to_remove = node
        if last_node_to_remove:
            for i in range(self.raw_path.index(last_node_to_remove) + 1):
                self.get_raw_path(0, pop=True)

    def collect_paths(self):
        self.set_raw_path([])
        self.set_path([])
        self.current_objective = None

    def register_game_state(self, origin, end):
        self.game_states.append([origin, end, self.path, self.large_obstacles, self.thin_obstacles, self.resolution, self.map_boundaries])
        with open("all_game_states.pkl", "wb") as file:
            pickle.dump(self.game_states, file)

    def wait_until_ready(self):
        while self.state_robot != READY:
            # rospy.loginfo("(STRATEGY) Waiting for the robot to be ready...")
            self.custom_waiting_rate.sleep()
            if self.phase_end():
                return

    def camera_reset_request(self, data: Bool):
        if data.data:
            self.reset_position_from_camera(wait=0.5, force=True)

    def reset_position_from_camera(self, wait: float = .3, force: bool = False):
        """Publishes the camera position to the odometry topic to correct the odometry"""
        if not force:
            self.wait_until_ready()
            if self.phase_end():
                return
        rospy.sleep(wait)
        if time.time() - self.last_time_cam < 2:
            self.got_cam_data = False
            while not self.got_cam_data:
                rospy.sleep(0.05)
            
            # Reposition the robot
            self.direction_pub.publish(Int16(BEST_DIRECTION))
            self.wait_until_ready()

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
        """Return True if all/any ids of bumpers are activated."""
        return any(self.bumpers & (1 << i) for i in ids)

    def back_until_bumper(self, speed: float = 0.2, axis: str = 'y+', direction: int = BACKWARD, shift: int = 10):
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

        while (abs(self.camera_position.theta - angle) > 0.15 and use_cam) or (
                abs(self.position.theta - angle) > 0.15 and not use_cam):
            rospy.loginfo(f"(STRATEGY) Correcting angle : {self.position.theta} -> {angle}")
            self.go_to(-1, -1, angle)
            use_cam = self.reset_position_from_camera()
        rospy.loginfo(f"(STRATEGY) Rotated to {angle} SUCCESSFULLY")

    def setup_subscribers(self):
        rospy.Subscriber('odometry', Pose2D, self.update_position)
        rospy.Subscriber('lidar_data', PoseArray, self.update_lidar_data)
        rospy.Subscriber('ultrasound_sensor_data', Float32MultiArray, self.update_us_data)
        rospy.Subscriber('camera', Float32MultiArray, self.update_camera)
        rospy.Subscriber('solar_aruco', Int8, self.update_solar_winner)
        rospy.Subscriber('bumper', Byte, self.update_bumpers)
        rospy.Subscriber('state', Int16, self.update_state)
        rospy.Subscriber('team', Bool, self.update_team)
        rospy.Subscriber('request_camera_update', Bool, self.camera_reset_request)

    # -- Callbacks --
    def update_bumpers(self, data: Byte):
        """Update the bumper states by reading the Byte message from the bumper topic."""
        self.bumpers = int(data.data)

    def update_position(self, data):
        self.position = data

    def update_lidar_data(self, data: PoseArray):
        raw = data
        self.lidar_data = [(raw.poses[i].position.x, raw.poses[i].position.y) for i in range(len(raw.poses))]

    def update_us_data(self, data):
        raw = data.data
        self.us_data = [(raw[i], raw[i + 1]) for i in range(0, len(raw), 2)]  # Unflatten the data

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

    def update_solar_winner(self, winner: Int8):
        if self.need_solar_winner:
            self.need_solar_winner = False
            self.latest_solar_winner = winner.data

    # -- Accessors --
    # Getters
    def get_path(self, index, pop=False):
        if self.path:
            return self.path[index] if not pop else self.path.pop(index)

    def get_raw_path(self, index, pop=False):
        if self.raw_path:
            return self.raw_path[index] if not pop else self.raw_path.pop(index)

    def get_objective(self, index, pop=False):
        if self.objectives:
            return self.objectives[index] if not pop else self.objectives.pop(index)

    # Setters
    def set_path(self, value):
        self.path = value

    def set_raw_path(self, value):
        self.raw_path = value

    def set_objectives(self, value):
        self.objectives = value

    # -- DEBUG --
    def debug_phase_goto(self):
        while not rospy.is_shutdown():
            self.go_to(1.5, 1, -1, MEDIUM_SPEED, FORWARD)
            self.wait_until_ready()
            self.go_to(.45, 1, -1, MEDIUM_SPEED, BACKWARD)
            self.wait_until_ready()

    def debug_phase_rotate(self):
        while not rospy.is_shutdown():
            rospy.loginfo("(STRATEGY) Rotating to 0")
            self.rotate_only(pi)
            self.wait_until_ready()


def run(data):
    global start
    start = data
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from RunningPhase")


if __name__ == "__main__":
    pr = cProfile.Profile() if PROFILE else None
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
        if PROFILE:
            pr.disable()
            s = io.StringIO()
            ps = pstats.Stats(pr, stream=s).sort_stats("cumulative")
            # Sauvegarde des résultats dans un fichier
            ps.print_stats()

            with open('cProfile.txt', 'w') as f:
                f.write(s.getvalue())
