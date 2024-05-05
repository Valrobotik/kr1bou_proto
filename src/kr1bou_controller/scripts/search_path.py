import heapq
import random
import time
from math import atan2
from utils import *
import numpy as np
import pickle as pkl


def a_star(start_node: Node, end_node: Node) -> Optional[List[Node]]:
    """
    A* Pathfinding algorithm implementation. Minimizes distance and changes of orientation
    :param start_node: the starting node / points
    :param end_node: the objective node / points
    :return: a list of nodes from start to end, or None if no path is found
    """
    open_set = [start_node]
    heapq.heapify(open_set)
    closed_set = set()

    start_node.g = 0
    start_node.h = heuristic(start_node, end_node)
    start_node.f = start_node.h

    # Loop until the open list is empty
    while open_set:
        # Use a priority queue to get the node with the lowest f value
        current_node = heapq.heappop(open_set)  # Also remove from open set
        closed_set.add(current_node)

        if current_node == end_node:  # Path found
            path = []
            current = current_node
            while current is not None:
                path.append(current)
                current = current.parent
            return path[::-1]

        # Explore neighbors
        for _, neighbor in current_node.neighbors.values():
            if neighbor.is_obstacle:  # WALL / OBSTACLE to ignore
                continue

            tentative_g = current_node.g + distance(current_node, neighbor)  # Distance from start to neighbor
            if tentative_g < neighbor.g:
                neighbor.parent = current_node
                neighbor.g = tentative_g
                neighbor.h = heuristic(neighbor, end_node)
                neighbor.f = neighbor.g + neighbor.h
                if neighbor not in closed_set:  # To be visited
                    heapq.heappush(open_set, neighbor)
    return None


def heuristic(node: Node, end_node: Node) -> float:
    """Heuristic function to estimate the cost of moving from the current node to the objective node"""
    # return other_estimate(node, end_node)
    return euclidian(node, end_node)
    # return manhattan(node, end_node)


def distance(node1: Node, node2: Node) -> float:
    """Distance between two nodes to be measured as the cost of moving from node1 to node2"""
    # return euclidian(node1, node2)
    # return manhattan(node1, node2)
    return 1 if node1.position[0] == node2.position[0] or node1.position[1] == node2.position[1] else 2
    # return 1


def orientation_change(node1: Node, node2: Node) -> float:
    """
    Compute the change in orientation between 2 nodes.
    :return: the angle between node1's and 2's orientation
    """
    # get angle between node1 and node2
    x1, y1 = node1.position
    x2, y2 = node2.position
    if x1 == x2:  # vertical line
        angle = 0 if y1 < y2 else pi  # 0 or 180 degrees
    elif y1 == y2:  # horizontal line
        angle = pi/2 if x1 < x2 else 3*pi/2  # 90 or 270 degrees
    else:  # diagonal line
        angle = atan2(y2 - y1, x2 - x1)  # angle in radians

    node2.orientation = angle  # update node2's orientation
    return abs(node1.orientation - node2.orientation)  # Compute the change


def clean_path(path: List[Node]) -> List[Node]:
    """Clean the path by removing intermediate nodes. Only keep the start, end, and turning points."""
    if not path:
        return []
    if len(path) < 3:
        return path
    cleaned_path = [path[0]]
    for i in range(1, len(path) - 1):
        if orientation_change(path[i - 1], path[i]) != orientation_change(path[i], path[i + 1]):
            cleaned_path.append(path[i])
    cleaned_path.append(path[-1])
    return cleaned_path


def generate_random_maze(height: int, width: int, density: float):
    """
    Generate a random maze
    :param height: height of the maze
    :param width: width of the maze
    :param density: density of the maze
    :return: a list of nodes representing the maze
    """
    maze = np.zeros((height, width), dtype=Node)
    maze = setup_maze(maze, set())

    i_j_to_remove = random.sample([(i, j) for i in range(height) for j in range(width)], int(density * height * width))
    for i, j in i_j_to_remove:
        maze[i][j].is_obstacle = True
    return maze


def generate_random_start_end(maze: List[List[Node]]) -> tuple:
    """
    Generate random start and end nodes
    :param maze: the maze
    :return: a tuple of start and end nodes
    """
    not_obstacle_nodes = [node for row in maze for node in row if not node.is_obstacle]
    start = random.choice(not_obstacle_nodes)
    end = random.choice(not_obstacle_nodes)
    return start, end


def verbose_print(condition, *args):
    if condition:
        print(*args)


def test_n(n: int = 1000, verbose: bool = False):
    times = []
    height, width, density, resolution = 200, 300, 0.4, 10
    for _ in range(n):
        maze = generate_random_maze(height, width, density)
        obstacles = {node.position for row in maze for node in row if node.is_obstacle}
        start, end = generate_random_start_end(maze)
        verbose_print(verbose, f"Start and End nodes: {start} {end}")
        onset = time.perf_counter()
        path = a_star(start, end)
        offset = time.perf_counter()
        execution_time = offset - onset
        times.append(execution_time)

        if path:
            cleaned_path = clean_path(path)
        else:
            cleaned_path = []

        if verbose:
            print(f"Path: {' '.join(map(str, cleaned_path)) if path else 'No path found'}")
            print(f"Start: {start}\nEnd: {end}\nDensity: {density}\nHeight: {height}\nWidth: {width}")
            print(f"Execution time: {execution_time} seconds\n")

        # Adjust positions and save the game state
        cleaned_path = meters_to_units(cleaned_path, resolution)
        map_boundaries = [0, 0, width / resolution, height / resolution]
        save_game_state(maze, cleaned_path, obstacles, resolution, map_boundaries, "maze.png", show=True)

    print(f"Average time: {sum(times) / n} seconds")


def debug():
    with open("/home/valrob/Desktop/all_game_states.pkl", "rb") as f:
        game_states = pkl.load(f)

    for count, (origin, end, path, obstacles1, obstacles2, resolution, boundaries) in enumerate(game_states):
        print(f"Game state {count}")
        print(f"Origin: {origin}\nEnd: {end}\nPath: {path}\nObstacles1: {len(obstacles1)}\nObstacles2: {len(obstacles2)}")
        print(f"Resolution: {resolution}\nBoundaries: {boundaries}")
        maze = setup_maze(np.zeros((boundaries[2] * resolution, boundaries[3] * resolution), dtype=Node), obstacles1)
        start = maze[origin[0]][origin[1]]
        end = maze[end[0]][end[1]]
        onset = time.perf_counter()
        path = a_star(start, end)
        offset = time.perf_counter()
        cleaned_path = clean_path(path) if path else []

        print(f"Maze size: {maze.shape}\nPath: {' '.join(map(str, cleaned_path)) if path else 'No path found'}")
        print(f"Start and End nodes: {start} {end}")
        print(f"Execution time: {offset - onset} seconds")

        # Adjust positions and save the game state
        cleaned_path = meters_to_units(cleaned_path, resolution)
        save_game_state(maze, cleaned_path, obstacles1, obstacles2, resolution, boundaries, f"archive/maze{count}.png", show=True)


if __name__ == '__main__':
    # test_n(1, True)
    # test_n(1000)
    debug()
    pass
