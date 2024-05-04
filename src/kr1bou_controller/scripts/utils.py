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


def get_discrete_obstacles(lidar_data: list, us_data: list, camera_data: list, resolution: int, radius: int, map_boundaries: list) -> set:
    """Get the obstacles from the ultrasound sensors (etc.), the position of the adversary and discretize them
    """
    obstacles = set()
    # Get the obstacles from the ultrasound sensors (values in meters)
    # obstacles = extend_obstacles(us_data, obstacles, radius, resolution, map_boundaries)
    # Get the obstacles from the lidar
    obstacles = extend_obstacles(lidar_data, obstacles, radius, resolution, map_boundaries)
    # Get the obstacles from the camera
    # obstacles = extend_obstacles(camera_data, obstacles, radius, resolution, map_boundaries)
    return obstacles


def extend_obstacles(data: list, obstacles: set, radius: int, resolution: int, map_boundaries: list) -> set:
    for i, (x, y) in enumerate(data):
        if (x, y) not in [(0, 0), (-1, -1)]:
            # Meters to unit
            x_, y_ = int(x * resolution), int(y * resolution)
            # Extend to a circle
            for j in range(-radius, radius + 1):
                for k in range(-radius, radius + 1):
                    if x_ + j < 0 or x_ + j >= map_boundaries[2] or y_ + k < 0 or y_ + k >= map_boundaries[3]:
                        continue
                    if j ** 2 + k ** 2 <= radius ** 2:  # Inside the circle
                        obstacles.add((x_ + j, y_ + k))
    return obstacles


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
            rospy.loginfo(f"Obstacle at {node.position}")
    return superposed == []


def clamp_theta(theta: float) -> float:
    """Clamp the angle between 0 and 2pi"""
    new_theta = theta
    if theta < 0:
        new_theta += 2 * pi
    return 2 * pi - new_theta
