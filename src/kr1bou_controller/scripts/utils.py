import rospy
from math import pi
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Optional, Tuple, Union

DIRECTIONS = {(1, 1), (1, 0), (1, -1), (0, 1), (0, -1), (-1, 1), (-1, 0), (-1, -1)}
INF = float('inf')


class Node:
    def __init__(self, position: tuple, orientation: float, neighbors=None, obstacle=False):
        if neighbors is None:
            neighbors = {}
        self.parent = None
        self.position = position
        self.orientation = orientation
        self.is_obstacle = obstacle
        self.neighbors = neighbors
        self.g = INF  # distance to previous position
        self.h = 0  # estimated distance
        self.f = 0

    def __lt__(self, other: 'Node'):
        return self.f < other.f

    def __eq__(self, other: 'Node'):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)

    def __str__(self):
        return f"Node({self.position}, {self.orientation})"

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
    def __init__(self, x, y, theta, direction):
        self.x = x
        self.y = y
        self.theta = theta
        self.direction = direction  # forward, backward, or best

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return f"Objective({self.x}, {self.y}, {self.theta})"

    def __repr__(self):
        return f"Objective({self.x}, {self.y}, {self.theta})"


def get_discrete_obstacles(lidar_data: list, us_data: list, camera_data: list, resolution: int, radius: int,
                           map_boundaries: list, position) -> Tuple[set, set]:
    """Get the obstacles from the ultrasound sensors (etc.), the position of the adversary and discretize them
    We extend each obstacle to 2 squares around it to take into account the robot's size. First square is radius, and
    second is more lenient. remove obstacles that are too close to position (10 cm)"""
    obstacles1, obstacles2 = set(), set()
    # Get the obstacles from the ultrasound sensors (values in meters)
    # obstacles1, obstacles2 = extend_obstacles(us_data, obstacles1, obstacles2, radius, resolution, map_boundaries)
    obstacles1, obstacles2 = extend_obstacles(lidar_data, obstacles1, obstacles2, radius, resolution, map_boundaries)
    obstacles1, obstacles2 = extend_obstacles(camera_data, obstacles1, obstacles2, radius, resolution, map_boundaries)
    # Remove obstacles that are too close to the robot
    x, y = int(position.x * resolution), int(position.y * resolution)  # Meters to unit
    for j in range(-10, 11):
        for k in range(-10, 11):
            if 0 <= x + j < map_boundaries[2] * resolution and 0 <= y + k < map_boundaries[3] * resolution:
                if (x + j, y + k) in obstacles1:
                    obstacles1.remove((x + j, y + k))
                if (x + j, y + k) in obstacles2:
                    obstacles2.remove((x + j, y + k))
    
    return obstacles1, obstacles2


def extend_obstacles(data: list, obstacles1: set, obstacles2: set, radius: int, resolution: int,
                     map_boundaries: list) -> Tuple[set, set]:
    if not data:
        return obstacles1, obstacles2
    for i, (x, y) in enumerate(data):
        x_, y_ = int(x * resolution), int(y * resolution)  # Meters to unit
        # Extend to a square of radius around the obstacle
        for j in range(-radius, radius + 1):
            for k in range(-radius, radius + 1):
                if 0 <= x_ + j < map_boundaries[2] * resolution and 0 <= y_ + k < map_boundaries[3] * resolution:
                    obstacles1.add((x_ + j, y_ + k))
                    # Extend to a square of radius - 10 around the obstacle
                    if -radius + 10 <= j <= radius - 10 and -radius + 10 <= k <= radius - 10:
                        obstacles2.add((x_ + j, y_ + k))
    return obstacles1, obstacles2


def bresenham(start: tuple, end: tuple) -> list:
    """Bresenham algorithm to get the path between two points."""
    x0, y0 = start
    x1, y1 = end
    points = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy  # error value e_xy

    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:  # e_xy+e_x > 0
            err += dy
            x0 += sx
        if e2 <= dx:  # e_xy+e_y < 0
            err += dx
            y0 += sy

    return points


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


def update_maze(maze: Union[np.ndarray, List[List[Node]]], new_obstacles: set, obstacles: set) -> Union[np.ndarray, List[List[Node]]]:
    not_obstacles_anymore = obstacles - new_obstacles
    for not_obstacle in not_obstacles_anymore:
        maze[not_obstacle[0]][not_obstacle[1]].is_obstacle = False

    for new_obstacle in new_obstacles:
        maze[new_obstacle[0]][new_obstacle[1]].is_obstacle = True

    for i in range(maze.shape[0]):
        for j in range(maze.shape[1]):
            maze[i][j].g = INF
            maze[i][j].f = 0
            maze[i][j].h = 0
            maze[i][j].parent = None
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
    return [Node((node.position[0] / resolution, node.position[1] / resolution), node.orientation) for node in path]


def parse_camera_data(own_robot, enemy_robot):
    if own_robot.x != -1 and own_robot.y != -1:
        return own_robot, enemy_robot
    if enemy_robot.x != -1 and enemy_robot.y != -1:
        return enemy_robot, own_robot
    return None, None


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


def save_game_state(maze, path: list, obstacles1: set, obstacles2, resolution: int, map_boundaries: list, filename: str, show=False):
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
    for obstacle in obstacles1:
        ax.add_patch(plt.Rectangle((obstacle[0], obstacle[1]), 1, 1, color='red'))
    for obstacle in obstacles2:
        ax.add_patch(plt.Rectangle((obstacle[0], obstacle[1]), 1, 1, color='yellow'))
    plt.savefig(filename)
    if show:
        plt.show()
    plt.close()
