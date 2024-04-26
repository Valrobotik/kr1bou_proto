#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64, Bool, Int16, Float32MultiArray, Byte
from search_path import Node, a_star

from math import sqrt
import heapq

READY_LINEAR = 0
READY = 1
IN_PROGRESS = 2

FORWARD = 1
BACKWARD = -1
BEST_DIRECTION = 0

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


class Strategy:
    def __init__(self) -> None:
        self.need_for_compute = False  # Whether to ask for a new path
        # Map boundaries in decimeters [x_min, y_min, x_max, y_max]. Example: [0, 0, 200, 300]
        self.unit = 10  # 1 dm = 10 cm
        self.map_boundaries = [ int(m / self.unit) for m in rospy.get_param('/map_boundaries') ]
        self.path = []  # List of waypoints to follow
        self.custom_waiting_rate = rospy.Rate(20)

        self.position = Pose2D()
        rospy.Subscriber("odometry", Pose2D, self.update_position)
        
        # Create a heapqueue based on the distance to the objectives
        self.objectives = [Objective(x, y, theta, sqrt((x - self.position.x * self.unit) ** 2 + (y - self.position.y * self.unit) ** 2)) for
                           x, y, theta in rospy.get_param('/objectives')]
        heapq.heapify(self.objectives)

        self.US_data = Float32MultiArray()
        rospy.Subscriber('ultrasound_sensor_data', Float32MultiArray, self.update_us_data)

        for i in range(1, 5):
            setattr(self, f'bumper_{i}', False)
        rospy.Subscriber('bumper', Byte, self.update_bumpers)

        self.state_robot = READY
        rospy.Subscriber('state', Int16, self.update_state)

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

    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo(self.position)
            if self.need_for_compute or self.path == []:
                if self.path == []:
                    rospy.loginfo(f"precision: {self.unit}")
                    if len(self.path) > 0 :
                        rospy.loginfo(f"Distance to objective: {sqrt((self.position.x * self.unit - self.path[0][0]) ** 2 + (self.position.y * self.unit - self.path[0][1]) ** 2)}")
                        if sqrt((self.position.x * self.unit - self.path[0][0]) ** 2 + (self.position.y * self.unit - self.path[0][1]) ** 2) < (0.5):
                            rospy.loginfo("[NEW]")
                            self.path.pop(0)
                if self.path and len(self.path) > 0:
                    rospy.loginfo(f"Computing path for {self.path[0]}")
                self.update_objectives()
                self.compute_path()
                self.need_for_compute = False
                rospy.loginfo(f"New path: {self.path}")
            if self.state_robot == READY:
                self.follow_path()
            else:
                rospy.loginfo(f" [state] : {self.state_robot}")
                self.wait_until_ready()

    def update_objectives(self):
        """Update heapqueue depending on the new position of the robot"""
        for obj in self.objectives:
            obj.cost = sqrt((obj.x - self.position.x * self.unit) ** 2 + (obj.y - self.position.y * self.unit) ** 2)
        heapq.heapify(self.objectives)

    def compute_path(self):
        """Aggregate all the data and compute the path to follow using A* algorithm. Neighbors are defined by a dict of
        the form {direction: (cost, neighbor_node)}. The cost is very high if the neighbor is an obstacle.
        :return: the path to follow
        """
        # Create a matrix of nodes
        maze = [[Node((x, y), {}) for y in range(self.map_boundaries[3])]
                for x in range(self.map_boundaries[2])]
        obstacles = self.get_discrete_obstacles()
        # Set the neighbors for each i, j. If any obstacle is in neighborhood, set the cost to MAX_COST
        for i in range(len(maze)):
            for j in range(len(maze[0])):
                if maze[i][j] is not None:
                    for direction in [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                        x, y = i + direction[0], j + direction[1]
                        if 0 <= x < len(maze) and 0 <= y < len(maze[0]) and maze[x][y]:
                            cost = MAX_COST if (x, y) in obstacles else 1
                            maze[i][j].neighbors[direction] = (cost, maze[x][y])
        # Get the start and end nodes
        origin = maze[int(self.position.x * self.unit)][int(self.position.y * self.unit)]
        if self.path == []:
            new_obj = heapq.heappop(self.objectives)  # Get new closest objective
        else :
            new_obj = self.objectives[0]
        # Compute the path. Remove first node
        path = a_star(origin, maze[int(new_obj.x)][int(new_obj.y)])[1:]
        # Remove node if the robot is already on it
        rospy.loginfo(f"dist to first cell : {(self.position.x * self.unit - path[0].position[0]) ** 2 + (self.position.y * self.unit - path[0].position[1]) ** 2}")
        if sqrt((self.position.x * self.unit - path[0].position[0]) ** 2 + (self.position.y * self.unit - path[0].position[1]) ** 2) < (0.7):
            rospy.loginfo("supression de la premierre cell")
            path.pop(0)
        self.path = [node.position for node in path]

    def get_discrete_obstacles(self) -> list:
        """Get the obstacles from the ultrasound sensors, the bumpers, the position of the adversary and discretize them
        """
        obstacles = []
        # Get the obstacles from the ultrasound sensors
        for i, (x, y) in enumerate(self.US_data.data):
            if (x, y) not in [(0, 0), (-1, -1)]:
                # Get the 4 points of the obstacle
                x_prime = int(x / self.unit)
                y_prime = int(y / self.unit)
                obstacles.extend([(x_prime, y_prime), (x_prime + 1, y_prime), (x_prime, y_prime + 1),
                                  (x_prime + 1, y_prime + 1)])
        # TODO : Remove testing default obstacles below
        obstacles.extend([(10, 7), (10, 14)])

        # TODO : Get the obstacles from the camera
        return obstacles

    def follow_path(self):
        """Follow the path"""
        if self.path and len(self.path) !=0 :
            self.go_to(self.path[0][0] / (self.unit), self.path[0][1] / (self.unit), -1, DEFAULT_MAX_SPEED, BEST_DIRECTION)
            rospy.loginfo(f"Going to ({self.path[0][0] / (self.unit)}, {self.path[0][1] / (self.unit)})")
        else :
            rospy.loginfo("no path found")


def run(data):
    global start
    start = data
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from RunningPhase")


if __name__ == "__main__":
    start = False
    rospy.init_node("strategy")
    rospy.loginfo("[START] Strategy node has started.")

    rospy.Subscriber('runningPhase', Bool, run)
    rate = rospy.Rate(rospy.get_param('/frequency'))
    while not start:
        rate.sleep()

    strategy_manager = Strategy()
    strategy_manager.run()
