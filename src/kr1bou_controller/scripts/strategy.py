#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64, Bool, Int16, Float32MultiArray, Byte
from search_path import Node, a_star, clean_path

from math import sqrt
import heapq

from typing import List

READY_LINEAR = 0
READY = 1
IN_PROGRESS = 2

FORWARD = 1
BACKWARD = -1
BEST_DIRECTION = 0

TEAM_BLUE = 1
TEAM_YELLOW = 0

DEFAULT_MAX_SPEED = 0.25

MAX_COST = 1000000


class Objective:
    def __init__(self, x, y, theta, cost):
        self.x = x
        self.y = y
        self.theta = theta
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return f"({self.x}, {self.y}, {self.theta}, {self.cost})"
    
    def __repr__(self):
        return f"Objective({self.x}, {self.y}, {self.theta}, {self.cost})"


class Strategy:
    def __init__(self) -> None:
        self.need_for_compute = True  # Whether to ask for a new path
        self.need_for_send = False

        self.next_pos_obj = [0, 0, 0]

        self.current_objective: Objective = None

        # Map boundaries in meters [x_min, y_min, x_max, y_max]. Example: [0, 0, 2, 3]
        self.map_boundaries = [int(m) for m in rospy.get_param('/map_boundaries') ]
        self.resolution = rospy.get_param('/resolution')  # Resolution to centimeters for example.
        self.path = []  # List of waypoints to follow
        self.custom_waiting_rate = rospy.Rate(20)
        

        self.position = Pose2D()
        rospy.Subscriber("odometry", Pose2D, self.update_position)
        
        # Create a heapqueue based on the distance to the objectives
        self.objectives = [Objective(x, y, theta, sqrt((x - self.position.x) ** 2 + (y - self.position.y) ** 2)) for
                           x, y, theta in rospy.get_param('/objectives')]
        heapq.heapify(self.objectives)

        self.US_data = [(-1, -1), (-1, -1), (-1, -1), (-1, -1), (-1, -1),(-1, -1), (-1, -1), (-1, -1), (-1, -1), (-1, -1)]
        rospy.Subscriber('ultrasound_sensor_data', Float32MultiArray, self.update_us_data)

        for i in range(1, 5):
            setattr(self, f'bumper_{i}', False)
        rospy.Subscriber('bumper', Byte, self.update_bumpers)

        self.state_robot = READY
        rospy.Subscriber('state', Int16, self.update_state)

        self.team = -1
        rospy.Subscriber('Team', Bool, self.update_team)
        
        self.solar_pub = rospy.Publisher('solar_angle', Int16, queue_size=1)
        self.pos_ordre_pub = rospy.Publisher('next_objectif', Pose2D, queue_size=1)
        self.direction_pub = rospy.Publisher('direction', Int16, queue_size=1)
        self.speed_ctrl_pub = rospy.Publisher('max_speed', Float64, queue_size=1)

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
        self.US_data = [(raw[i], raw[i + 1]) for i in range(0, len(raw), 2)]
        self.need_for_compute = True
    
    def update_team(self, data:Bool):
        if self.team == -1 :
            if data.data : 
                self.team = TEAM_BLUE
            else :
                self.team = TEAM_YELLOW
        else : 
            rospy.logwarn("(STRATEGY) /!\\ CAN NOT CHANGE TEAM DURING THE MATCH /!\\")
    
    def update_state(self, data: Int16):
        self.state_robot = data.data
        self.need_for_compute = True

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
        rospy.sleep(0.01)
        self.speed_ctrl_pub.publish(speed_data)
        rospy.sleep(0.01)
        self.pos_ordre_pub.publish(obj)

    def turn_servo(self, alpha):
        a = Int16()
        a.data = alpha
        self.solar_pub.publish(a)

    def wait_until_ready(self):
        while self.state_robot != READY:
            self.custom_waiting_rate.sleep()
            if (sqrt((self.next_pos_obj[0]-self.position.x)**2+(self.next_pos_obj[1]-self.position.y)**2) < 0.07 and self.next_pos_obj[2] == -1) : 
                rospy.loginfo("(STRATEGY) Robot is close enough to the node. Waiting for the next order.")
                rospy.loginfo(f"(STRATEGY) Current dist : {sqrt((self.next_pos_obj[0]-self.position.x)**2+(self.next_pos_obj[1]-self.position.y)**2)}")
                rospy.loginfo(f"(STRATEGY) Threshold : 0.07")
                break
        self.need_for_compute = True

    def run(self):
        rospy.loginfo("(STRATEGY) Strategy running loop has started.")
        while not rospy.is_shutdown():
            while self.team == -1 and not rospy.is_shutdown(): rospy.sleep(0.05)
            if self.need_for_compute:   # New sensor data
                if len(self.path) > 0 :  # If the robot is already following a path
                    rospy.loginfo(f"(STRATEGY) Distance : {sqrt((self.position.x - self.path[0].position[0]) ** 2 + (self.position.y - self.path[0].position[1]) ** 2)}")
                    rospy.loginfo(f"(STRATEGY) Threshold : {5.0 / self.resolution}")
                    while sqrt((self.position.x - self.path[0].position[0]) ** 2 + (self.position.y - self.path[0].position[1]) ** 2) < 5.0 / self.resolution: # example : 5 cm
                        rospy.loginfo(f"(STRATEGY) Robot is close enough to the nearest waypoint. Removing {self.path[0]} from the path.")
                        self.path.pop(0)  # Remove if he is close enough to the current intermediate objective

                # get new path
                self.update_objectives() # update heapqueue

                rospy.loginfo(f"(STRATEGY) Objectives : {self.objectives}")
                self.compute_path()
                rospy.loginfo(f"(STRATEGY) Path : {self.path}")
                self.need_for_compute = False
                self.follow_path()
            else:
                self.wait_until_ready()

    def update_objectives(self):
        """Update heapqueue depending on the new position of the robot"""
        for obj in self.objectives:
            obj.cost = sqrt((obj.x - self.position.x) ** 2 + (obj.y - self.position.y) ** 2)
        heapq.heapify(self.objectives)

    def update_team(self, data:Bool):
        global start
        if not start or self.team != -1:
            if data.data : 
                self.team = TEAM_BLUE
            else :
                self.team = TEAM_YELLOW
        else :
            rospy.loginfo("(STRATEGY) YOU CAN'T CHANGE TEAM AFTER STARTING THE GAME !")

    def compute_path(self):
        """Aggregate all the data and compute the path to follow using A* algorithm. Neighbors are defined by a dict of
        the form {direction: (cost, neighbor_node)}. The cost is very high if the neighbor is an obstacle.
        :return: the path to follow
        """
        rospy.loginfo("(STRATEGY) IN COMPUTE PATH FUNCTION")
        # Create a matrix of nodes
        maze = [[Node((x, y), 0, {}) for y in range(int(self.map_boundaries[3] * self.resolution))]
                for x in range(int(self.map_boundaries[2] * self.resolution))]
        rospy.loginfo("(STRATEGY) Maze created")
        obstacles = self.get_discrete_obstacles()
        rospy.loginfo(f"(STRATEGY) Obstacles : {obstacles}")
        
        # Set the neighbors for each i, j. 
        for i in range(len(maze)):
            for j in range(len(maze[0])):
                if maze[i][j] is not None:
                    for direction in [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                        x, y = i + direction[0], j + direction[1]
                        if 0 <= x < len(maze) and 0 <= y < len(maze[0]) and maze[x][y]: # If any obstacle is in neighborhood,
                            cost = MAX_COST if (x, y) in obstacles else 1               # set the cost to MAX_COST
                            maze[i][j].neighbors[direction] = (cost, maze[x][y])
        rospy.loginfo("(STRATEGY) Neighbors set")
        
        # Get the start and end nodes
        origin = maze[int(self.position.x * self.resolution)][int(self.position.y * self.resolution)]
        origin.orientation = self.position.theta
        if self.path == []:
            self.current_objective  = heapq.heappop(self.objectives)  # Get new closest objective

        rospy.loginfo(f"(STRATEGY) Current start/end : {origin.position}/{self.current_objective}")
        print("Maze :")
        # print_maze(origin, Node((int(new_obj.x * self.resolution), int(new_obj.y * self.resolution)), 0), maze)
        
        # Compute the path
        if self.is_path_valid():
            rospy.loginfo("(STRATEGY) Path still exists")
            path = self.path # Keep the current path
        else:
            rospy.loginfo("(STRATEGY) Recompute path")
            # apply resolution 
            path = a_star(origin, maze[int(self.current_objective.x * self.resolution)][int(self.current_objective.y * self.resolution)])[1:] # Remove current position node
            path = clean_path(path)
            rospy.loginfo(f"(STRATEGY) New path : {path}")
        
        # Remove node if the robot is already on it
        if len(path) > 0:
            rospy.loginfo(f"(STRATEGY) Distance : {sqrt((self.position.x * self.resolution - path[0].position[0]) ** 2 + (self.position.y * self.resolution - path[0].position[1]) ** 2)}")
            rospy.loginfo(f"(STRATEGY) Threshold : {7}")
            while sqrt((self.position.x * self.resolution - path[0].position[0]) ** 2 + (self.position.y * self.resolution - path[0].position[1]) ** 2) < 7: # example : 7 cm
                path.pop(0)
        self.path = [Node((node.position[0] / self.resolution,node.position[1] / self.resolution), node.orientation) for node in path]

    def get_discrete_obstacles(self) -> list:
        """Get the obstacles from the ultrasound sensors, the bumpers, the position of the adversary and discretize them
        """
        obstacles = []
        # Get the obstacles from the ultrasound sensors (values in meters)
        # for i, (x, y) in enumerate(self.US_data):
        #     if (x, y) not in [(0, 0), (-1, -1)]:
        #         # Extend to a circle of radius 10 cm
        #         for j in range(-int(0.1 * self.resolution), int(0.1 * self.resolution) + 1):
        #             for k in range(-int(0.1 * self.resolution), int(0.1 * self.resolution) + 1):
        #                 if sqrt(j ** 2 + k ** 2) <= 0.1 * self.resolution:
        #                     obstacles.append((int(x * self.resolution) + j, int(y * self.resolution) + k))
        # TODO : Remove testing default obstacles below
        # obstacles.extend([(100, 70), (100, 140)])

        # TODO : Get the obstacles from the camera
        return obstacles
    
    def is_path_valid(self):
        """Check if the current path is still valid, i.e no obstacles on the path"""
        if self.path == []:
            return False
        superposed = []
        for node in self.path:
            if node.position in self.get_discrete_obstacles():
                superposed.append(node.position)
            rospy.loginfo(f"Obstacle at {node.position}")
        return superposed == []

    def follow_path(self):
        """Follow the path"""
        if self.path and len(self.path) !=0 :
            rospy.loginfo(f"(STRATEGY) Following path : {self.path}")
            self.go_to(self.path[0].position[0], self.path[0].position[1], -1, DEFAULT_MAX_SPEED, BEST_DIRECTION)
            rospy.loginfo(f"(STRATEGY) Going to {self.path[0]}")
        else :
            rospy.loginfo("(STRATEGY) No path found")


def run(data):
    global start
    start = data
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from RunningPhase")


if __name__ == "__main__":
    start = False
    rospy.init_node("strategy")
    rospy.loginfo("[START] Strategy node has started.")
    strategy_manager = Strategy()

    rospy.Subscriber('runningPhase', Bool, run)
    rate = rospy.Rate(rospy.get_param('/frequency'))
    while not start:
        rate.sleep()

    strategy_manager.run()
