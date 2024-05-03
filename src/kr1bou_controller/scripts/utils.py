import rospy
# from math import sqrt
from search_path import Node
from math import pi
import numpy as np

DIRECTIONS = {(1, 1), (1, 0), (1, -1), (0, 1), (0, -1), (-1, 1), (-1, 0), (-1, -1)}


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


def get_discrete_obstacles(lidar_data: list, us_data: list, resolution: int) -> list:
    """Get the obstacles from the ultrasound sensors, the bumpers, the position of the adversary and discretize them
    """
    obstacles = []
    # Get the obstacles from the ultrasound sensors (values in meters)
    for i, (x, y) in enumerate(us_data):
        if (x, y) not in [(0, 0), (-1, -1)]:
            # TODO : Extend to a circle
            pass

    # TODO : Remove testing default obstacles below
    obstacles.extend([(100, 70), (100, 140)])

    # TODO : Get the obstacles from the camera
    return obstacles


def setup_maze(shape: tuple, obstacles: set) -> list:
    """Create the maze with the obstacles"""
    # update obstacles in the maze
    maze = np.zeros(shape, dtype=Node)
    rospy.loginfo(f"start_1")
    for i in range(shape[0]):
        for j in range(shape[1]):
            maze[i][j] = Node((i, j), 0, obstacle=((i, j) in obstacles))
    for i in range(shape[0]):
        for j in range(shape[1]):
            for direction in DIRECTIONS:
                x = i + direction[0]
                y = j + direction[1]
                if 0 <= x < shape[0] and 0 <= y < shape[1]:
                    maze[i][j].neighbors.append(maze[x][y])
    rospy.loginfo(f"end_1")
    return maze

def update_maze(maze: list, obstacles: set, new_obstacles: set) -> list:
    rospy.loginfo(f"2")
    not_obstacles_anymore = obstacles - new_obstacles
    rospy.loginfo(f"3")
    for not_obstacle in not_obstacles_anymore:
        maze[not_obstacle[0]][not_obstacle[1]].obstacle = False
    rospy.loginfo(f"4")
    for new_obstacle in new_obstacles:
        maze[new_obstacle[0]][new_obstacle[1]].obstacle = True
    rospy.loginfo(f"5")
    return maze


def is_path_valid(path: list, obstacles: set) -> bool:
    """Check if the current path is still valid, i.e. no obstacles on the path"""
    if not path:
        return False
    superposed = []
    for node in path:
        if node.position in obstacles:
            superposed.append(node.position)
            rospy.loginfo(f"Obstacle at {node.position}")
    return superposed == []


def clamp_theta(theta: float) -> float:
    """Clamp the angle between 0 and 2pi"""
    new_theta = theta
    if theta < 0:
        new_theta += 2 * pi
    return 2*pi - new_theta
