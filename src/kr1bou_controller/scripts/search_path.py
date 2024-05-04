"""
A* Pathfinding algorithm implementation
"""

from typing import List, Optional
import heapq
import random
import time
from math import atan2, pi
from utils import Node, save_game_state, setup_maze, print_maze, manhattan, euclidian, other_estimate
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
    """
    Heuristic function
    :param node: current node
    :param end_node: objective node
    :return: heuristic value
    """
    # return other_estimate(node, end_node)
    return euclidian(node, end_node)
    # return manhattan(node, end_node)


def distance(node1: Node, node2: Node) -> float:
    """
    Distance between two nodes
    :param node1: node 1
    :param node2: node 2
    :return: distance between the two nodes
    """
    # return euclidian(node1, node2)
    # return manhattan(node1, node2)
    return 1 if node1.position[0] == node2.position[0] or node1.position[1] == node2.position[1] else 2
    # return 1


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
        obstacles = set([node.position for row in maze for node in row if node.is_obstacle])
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
    with open("/home/kribou/Downloads/all_game_states.pkl", "rb") as f:
        game_states = pkl.load(f)
    count = 0
    for origin, end, path, obstacles, resolution, boundaries in game_states[1:]:
        count += 1
        # replace obstacles by a square of radius 40 around 140, 50
        obstacles = set([(i, j) for i in range(100, 180) for j in range(10, 90)])
        maze = setup_maze(np.zeros((boundaries[2] * resolution, boundaries[3] * resolution), dtype=Node), obstacles)
        print(maze.shape)
        print(path)
        print(origin, end)
        start = maze[origin[0]][origin[1]]
        end = maze[end[0]][end[1]]
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
        path = cleaned_path if path else []
        print(path)
        if path:
            for node in path:
                node.position = (node.position[0] / resolution, node.position[1] / resolution)
        save_game_state(maze, path, obstacles, resolution, boundaries, f"archive/maze{count}.png", show=True)


if __name__ == '__main__':
    # test_n(1, True)
    # test_n(1000)
    debug()
