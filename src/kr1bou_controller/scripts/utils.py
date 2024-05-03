import rospy
# from math import sqrt
from search_path import Node
from math import pi

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


def setup_maze(maze: list, obstacles: set) -> list:
    """Create the maze with the obstacles"""
    # update obstacles in the maze
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            maze[i][j] = Node((i, j), 0, {}) if (i, j) not in obstacles else None
    # update neighbors
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if maze[i][j] is not None:
                for direction in DIRECTIONS - obstacles:
                    x, y = i + direction[0], j + direction[1]
                    if 0 <= x < len(maze) and 0 <= y < len(maze[0]) and maze[x][y] is not None:
                        maze[i][j].neighbors[direction] = (1, maze[x][y])
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
    if theta < 0:
        new_theta = 2 * pi + theta
    elif theta >= 2 * pi:
        new_theta = theta - 2 * pi
    else:
        new_theta = theta
    return new_theta
