import rospy
# from math import sqrt
from math import pi
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Optional, Tuple

DIRECTIONS = {(1, 1), (1, 0), (1, -1), (0, 1), (0, -1), (-1, 1), (-1, 0), (-1, -1)}
INF = float('inf')


class Node:
    def __init__(self, position: tuple, orientation: float, neighbors=None, obstacle=False):
        if neighbors is None:
            neighbors = {}
        self.position = position
        self.orientation = orientation
        self.neighbors = neighbors
        self.g = INF  # distance to previous position
        self.h = 0  # estimated distance
        self.f = 0
        self.parent = None
        self.is_obstacle = obstacle

    def __lt__(self, other: 'Node'):
        return self.f < other.f

    def __eq__(self, other: 'Node'):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)

    def __str__(self):
        return f"({self.position})"

    def __repr__(self):
        return f"Node({self.position}, {self.orientation})"


def euclidian(node1: Node, node2: Node) -> float:
    """
    Euclidian distance between two nodes
    :param node1: node 1
    :param node2: node 2
    :return: euclidian distance between the two nodes
    """
    return ((node1.position[0] - node2.position[0]) ** 2 + (node1.position[1] - node2.position[1]) ** 2) ** 0.5


def manhattan(node1: Node, node2: Node) -> float:
    """
    Manhattan distance between two nodes.
    :param node1: node 1
    :param node2: node 2
    :return: manhattan distance between the two nodes
    """
    return abs(node1.position[0] - node2.position[0]) + abs(node1.position[1] - node2.position[1])


def other_estimate(node1: Node, node2: Node) -> float:
    """
    Another way to measure distance between two nodes.
    :param node1: node 1
    :param node2: node 2
    :return: estimated distance between the two nodes
    """
    node_x, node_y = node1.position
    goal_x, goal_y = node2.position
    dx = abs(node_x - goal_x)
    dy = abs(node_y - goal_y)
    return dx + dy + (2 ** 0.5 - 2) * min(dx, dy)


class Objective:
    def __init__(self, x, y, theta, cost, direction):
        self.x = x
        self.y = y
        self.theta = theta
        self.cost = cost
        self.direction = direction

    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return f"({self.x}, {self.y}, {self.theta}, {self.cost})"

    def __repr__(self):
        return f"Objective({self.x}, {self.y}, {self.theta}, {self.cost})"


def get_discrete_obstacles(lidar_data: list, us_data: list, camera_data: list, resolution: int, radius: int,
                           map_boundaries: list) -> Tuple[set, set]:
    """Get the obstacles from the ultrasound sensors (etc.), the position of the adversary and discretize them
    We extend each obstacle to 2 squares around it to take into account the robot's size. First square is radius, and
    second is more lenient."""
    obstacles1, obstacles2 = set(), set()
    # Get the obstacles from the ultrasound sensors (values in meters)
    # obstacles1, obstacles2 = extend_obstacles(us_data, obstacles1, obstacles2, radius, resolution, map_boundaries)
    # Get the obstacles from the lidar
    obstacles1, obstacles2 = extend_obstacles(lidar_data, obstacles1, obstacles2, radius, resolution, map_boundaries)
    # Get the obstacles from the camera
    obstacles1, obstacles2 = extend_obstacles(camera_data, obstacles1, obstacles2, radius, resolution, map_boundaries)
    return obstacles1, obstacles2


def extend_obstacles(data: list, obstacles1: set, obstacles2: set, radius: int, resolution: int,
                     map_boundaries: list) -> Tuple[set, set]:
    if not data:
        return obstacles1, obstacles2
    for i, (x, y) in enumerate(data):
        if (x, y) not in [(0, 0), (-1, -1)]:
            # Meters to unit
            x_, y_ = int(x * resolution), int(y * resolution)
            # Extend to a square of radius around the obstacle
            for j in range(-radius, radius + 1):
                for k in range(-radius, radius + 1):
                    if 0 <= x_ + j < map_boundaries[2] * resolution and 0 <= y_ + k < map_boundaries[3] * resolution:
                        obstacles1.add((x_ + j, y_ + k))
                    # Extend to a square of radius - 10 around the obstacle
                    if -radius + 10 <= j <= radius - 10 and -radius + 10 <= k <= radius - 10:
                        obstacles2.add((x_ + j, y_ + k))
    return obstacles1, obstacles2


def setup_maze(maze, obstacles: set):
    """Create the maze with the obstacles"""
    for i in range(maze.shape[0]):
        for j in range(maze.shape[1]):
            maze[i][j] = Node((i, j), 0, obstacle=((i, j) in obstacles))
    for i in range(maze.shape[0]):
        for j in range(maze.shape[1]):
            for direction in DIRECTIONS:
                x = i + direction[0]
                y = j + direction[1]
                if 0 <= x < maze.shape[0] and 0 <= y < maze.shape[1]:
                    maze[i][j].neighbors[direction] = (1, maze[x][y])
    return maze


def update_maze(maze: np.ndarray, obstacles: set, new_obstacles: set):
    not_obstacles_anymore = obstacles - new_obstacles
    for not_obstacle in not_obstacles_anymore:
        maze[not_obstacle[0]][not_obstacle[1]].obstacle = False
    for new_obstacle in new_obstacles:
        maze[new_obstacle[0]][new_obstacle[1]].obstacle = True
    return maze


def is_path_valid(path: list, obstacles: set) -> bool:
    """Check if the current path is still valid, i.e. no obstacles on the path"""
    if not path:
        return False
    superposed = []
    for node in path:
        if node.position in obstacles:
            superposed.append(node.position)
    rospy.loginfo(f"Obstacle at {superposed}") if superposed else None
    return superposed == []


def meters_to_units(path: list, resolution: int) -> list:
    """Convert the path from meters to units"""
    return [(node.position[0] / resolution, node.position[1] / resolution) for node in path]


def clamp_theta(theta: float) -> float:
    """Clamp the angle between 0 and 2pi"""
    new_theta = theta
    if theta < 0:
        new_theta += 2 * pi
    return 2 * pi - new_theta


def print_maze(start, end, maze, path: Optional[List[Node]] = None):
    """
    Print the maze
    :param start: the start node
    :param end: the end node
    :param maze: the maze
    :param path: the path to be printed
    """
    for i in range(maze.shape[0]):
        for j in range(maze.shape[1]):
            node = maze[i][j]
            # Discriminate Obstacles, Nodes, and Path
            if node.is_obstacle:
                print("X", end=" ")
            elif node == start:
                print("S", end=" ")
            elif node == end:
                print("E", end=" ")
            elif path and node in path:
                print(path.index(node), end=" ")
            else:
                print(".", end=" ")
        print()


def save_game_state(maze, path: list, obstacles: set, resolution: int, map_boundaries: list, filename: str, show=False):
    """Save the current game state in a file using matplotlib"""
    fig, ax = plt.subplots()
    ax.set_xlim(0, map_boundaries[2] * resolution)  # Convert to unit
    ax.set_ylim(0, map_boundaries[3] * resolution)
    ax.set_aspect('equal')
    for i in range(maze.shape[0]):
        for j in range(maze.shape[1]):
            if maze[i][j].is_obstacle:
                ax.add_patch(plt.Rectangle((i, j), 1, 1, color='black'))
    if path:
        for node in path:  # convert path to used unit
            ax.add_patch(
                plt.Rectangle((node.position[0] * resolution, node.position[1] * resolution), 1, 1, color='blue'))
    for obstacle in obstacles:
        ax.add_patch(plt.Rectangle((obstacle[0], obstacle[1]), 1, 1, color='red'))
    plt.savefig(filename)
    if show:
        plt.show()
    plt.close()
