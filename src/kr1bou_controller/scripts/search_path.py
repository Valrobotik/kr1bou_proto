"""
A* Pathfinding algorithm implementation
"""

from typing import List, Optional
import heapq
import random
import time
from math import atan2, pi
from utils import Node, save_game_state, setup_maze, print_maze
import numpy as np
import pickle as pkl

gamma = 1  # 1 for now (180/pi later)


def a_star(start_node: Node, end_node: Node) -> Optional[List[Node]]:
    """
    A* Pathfinding algorithm implementation. Minimizes distance and changes of orientation
    :param start_node: the starting node / points
    :param end_node: the objective node / points
    :return: a list of nodes from start to end, or None if no path is found
    """
    open_list = [start_node]
    closed_list = set()

    # Loop until the open list is empty
    while open_list:
        # Use a priority queue to get the node with the lowest f value
        current_node = heapq.heappop(open_list)

        if current_node == end_node:  # Path found
            path = []
            current = current_node
            # print("Path found with cost: f =", current.f, "g =", current.g, "h =", current.h, "o =", current.o)
            while current is not None:
                path.append(current)
                current = current.parent
            return path[::-1]

        closed_list.add(current_node)  # Mark as visited

        # Explore neighbors
        for cost, neighbor in current_node.neighbors.values():
            if neighbor.is_obstacle:  # WALL / OBSTACLE to ignore
                continue

            # Compute the new cost
            neighbor.g = current_node.g + 1  # update g value
            neighbor.h = heuristic(neighbor, end_node)
            # neighbor.o = orientation_change(current_node, neighbor)
            # f = alpha * g + beta * h + gamma * o. Here alpha = cost, beta = 1, gamma -> ~radians to degrees
            neighbor.f = neighbor.g + neighbor.h  # + gamma * neighbor.o

            if neighbor in closed_list:  # Skip if already visited
                continue

            if neighbor not in open_list:  # To be visited
                heapq.heappush(open_list, neighbor)
                neighbor.parent = current_node
    
    return None


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


def other_heuristic(node1: Node, node2: Node) -> float:
    """
    Another way to measure distance between two nodes.
    :param node1: node 1
    :param node2: node 2
    :return: manhattan distance between the two nodes
    """
    node_x, node_y = node1.position
    goal_x, goal_y = node2.position
    dx = abs(node_x - goal_x)
    dy = abs(node_y - goal_y)
    return dx + dy + (2 ** 0.5 - 2) * min(dx, dy)


def heuristic(node: Node, end_node: Node) -> float:
    """
    Heuristic function
    :param node: current node
    :param end_node: objective node
    :return: heuristic value
    """
    # return other_heuristic(node, end_node)
    return euclidian(node, end_node)
    # return manhattan(node, end_node)


def orientation_change(node1: Node, node2: Node) -> float:
    """
    Compute the change in orientation between 2 nodes.
    :param node1: current node
    :param node2: next node
    :return: the angle between the orientation of node1 and the segment between node1 and node2
    """
    # get angle between node1 and node2
    x1, y1 = node1.position
    x2, y2 = node2.position
    if x1 == x2:
        if y1 < y2:
            angle = 0
        else:
            angle = pi
    elif y1 == y2:
        if x1 < x2:
            angle = pi/2
        else:
            angle = 3*pi/2
    else:
        angle = atan2(y2 - y1, x2 - x1)
    
    # update node2's orientation
    node2.orientation = angle
    return abs(node1.orientation - node2.orientation)  # Compute the change


def clean_path(path: List[Node]) -> List[Node]:
    """
    Clean the path by removing intermediate nodes. Only keep the start, end, and turning points.
    :param path: the path to clean
    :return: the cleaned path
    """
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
    
    # take a random number of nodes to remove
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
    # Go through each node and list them. Then take a random node from the list
    not_obstacle_nodes = [node for row in maze for node in row if not node.is_obstacle]
    start = random.choice(not_obstacle_nodes)
    end = random.choice(not_obstacle_nodes)
    return start, end


def test_n(n: int = 1000, verbose: bool = False):
    times = []
    height = 200
    width = 300
    density = 0.1
    resolution = 10
    for _ in range(n):
        maze = generate_random_maze(height, width, density)
        obstacles = [node.position for row in maze for node in row if node.is_obstacle]
        start, end = generate_random_start_end(maze)
        print("Start and End nodes:", start, end) if verbose else None
        onset = time.perf_counter()
        path = a_star(start, end)
        offset = time.perf_counter()
        # print_maze(start, end, maze, path) if verbose else None
        print() if verbose else None
        print("Path:") if verbose else None
        if path and verbose:
            for node in path:
                print(node, end=" ")
            cleaned_path = clean_path(path)
            for node in cleaned_path:
                print(node, end=" ")
            print()
        elif verbose:
            print("No path found")
        if verbose:
            print()
            print("Start:", start)
            print("End:", end)
            print("Density:", density)
            print("Height:", height)
            print("Width:", width)
            print("Execution time:", offset - onset, "seconds")
            print()
            path = cleaned_path if path else []
            for node in path:
                node.position = (node.position[0] / resolution, node.position[1] / resolution)
            save_game_state(maze, path, obstacles, resolution, [0, 0, width / resolution, height / resolution], "maze.png", show=True)
        times.append(offset - onset)
    print("Average time:", sum(times) / n, "seconds")


def debug():
    # load path, obstacles, resolution, boundaries from pickle
    with open("/home/kribou/Downloads/game_states.pkl", "rb") as f:
        game_states = pkl.load(f)
    count = 0
    for origin, end, path, obstacles, resolution, boundaries in game_states:
        count += 1
        maze = setup_maze(np.zeros((boundaries[2] * resolution, boundaries[3] * resolution), dtype=Node), obstacles)
        start = Node(origin, 0)
        end = Node(end, 0)
        print("Start and End nodes:", start, end)
        onset = time.perf_counter()
        path = a_star(start, end)
        offset = time.perf_counter()
        print("Path:")
        if path:
            for node in path:
                print(node, end=" ")
            cleaned_path = clean_path(path)
            for node in cleaned_path:
                print(node, end=" ")
            print()
        else:
            print("No path found")
        print("Execution time:", offset - onset, "seconds")
        print()
        path = cleaned_path if path else []
        for node in path:
            node.position = (node.position[0] / resolution, node.position[1] / resolution)
        save_game_state(maze, path, obstacles, resolution, boundaries, f"maze{count}.png", show=True)


if __name__ == '__main__':
    # test_n(1, True)
    # test_n(1000)
    debug()
