"""
A* Pathfinding algorithm implementation
"""

from typing import List, Optional
import heapq
import random
import time


class Node:
    """
    Node class
    """

    def __init__(self, position: tuple, neighbors: dict):
        self.position = position
        self.neighbors = neighbors
        self.g = 0
        self.h = 0
        self.f = 0
        self.parent = None

    def __lt__(self, other: 'Node'):
        return self.f < other.f

    def __eq__(self, other: 'Node'):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)

    def __str__(self):
        return f"({self.position})"


def a_star(start_node: Node, end_node: Node) -> Optional[List[Node]]:
    """
    A* Pathfinding algorithm implementation
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
            while current is not None:
                path.append(current)
                current = current.parent
            return path[::-1]

        closed_list.add(current_node)  # Mark as visited

        # Explore neighbors
        for cost, neighbor in current_node.neighbors.values():
            if neighbor is None:  # WALL / OBSTACLE
                continue

            # Compute the new cost
            neighbor.g = current_node.g + euclidian(neighbor, current_node)  # update g value
            neighbor.h = heuristic(neighbor, end_node)
            neighbor.f = cost * neighbor.g + neighbor.h  # f = alpha * g + beta * h. Here alpha = cost and beta = 1

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


def heuristic(node: Node, end_node: Node) -> float:
    """
    Heuristic function
    :param node: current node
    :param end_node: objective node
    :return: heuristic value
    """
    # node_x, node_y = node.position
    # goal_x, goal_y = end_node.position
    # dx = abs(node_x - goal_x)
    # dy = abs(node_y - goal_y)
    # return dx + dy + (2 ** 0.5 - 2) * min(dx, dy)
    return euclidian(node, end_node)


def generate_random_maze(height: int, width: int, density: float) -> List[List[Node]]:
    """
    Generate a random maze
    :param height: height of the maze
    :param width: width of the maze
    :param density: density of the maze
    :return: a list of nodes representing the maze
    """
    maze = [[Node((i, j), {}) for j in range(width)] for i in range(height)]
    for i in range(height):
        for j in range(width):
            if i > 0:
                maze[i][j].neighbors['N'] = (1, maze[i - 1][j])
            if i < height - 1:
                maze[i][j].neighbors['S'] = (1, maze[i + 1][j])
            if j > 0:
                maze[i][j].neighbors['W'] = (1, maze[i][j - 1])
            if j < width - 1:
                maze[i][j].neighbors['E'] = (1, maze[i][j + 1])
            if i > 0 and j > 0:
                maze[i][j].neighbors['NW'] = (1, maze[i - 1][j - 1])
            if i > 0 and j < width - 1:
                maze[i][j].neighbors['NE'] = (1, maze[i - 1][j + 1])
            if i < height - 1 and j > 0:
                maze[i][j].neighbors['SW'] = (1, maze[i + 1][j - 1])
            if i < height - 1 and j < width - 1:
                maze[i][j].neighbors['SE'] = (1, maze[i + 1][j + 1])

    not_none_nodes = [node for row in maze for node in row if node is not None]
    x_y_to_remove = random.sample(not_none_nodes, int(density * len(not_none_nodes)))
    # remove a random number of nodes according to the density
    for node in x_y_to_remove:
        # remove neighbors
        for neighbor in node.neighbors.values():
            cost, neighbor_node = neighbor
            if neighbor_node:
                for direction, neighbor_neighbor in neighbor_node.neighbors.items():
                    if neighbor_neighbor[1] and node and neighbor_neighbor[1] == node:
                        neighbor_node.neighbors[direction] = (cost, None)
        i, j = node.position
        maze[i][j] = None
    return maze


def generate_random_start_end(maze: List[List[Node]]) -> tuple:
    """
    Generate random start and end nodes
    :param maze: the maze
    :return: a tuple of start and end nodes
    """
    # Go through each node and list them. Then take a random node from the list
    not_none_nodes = [node for row in maze for node in row if node is not None]
    start = random.choice(not_none_nodes)
    end = random.choice(not_none_nodes)
    return start, end


def print_maze(start, end, maze: List[List[Node]], path: Optional[List[Node]] = None):
    """
    Print the maze
    :param start: the start node
    :param end: the end node
    :param maze: the maze
    :param path: the path to be printed
    """
    for row in maze:
        for node in row:
            # Discriminate None, Nodes, and Path
            if node is None:
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


def test_n(n: int = 1000, verbose: bool = False):
    times = []
    height = 20
    width = 30
    density = 0.2
    for _ in range(n):
        maze = generate_random_maze(height, width, density)
        start, end = generate_random_start_end(maze)
        print("Start and End nodes:", start, end) if verbose else None
        onset = time.perf_counter()
        path = a_star(start, end)
        offset = time.perf_counter()
        print_maze(start, end, maze, path) if verbose else None
        print() if verbose else None
        print("Path:") if verbose else None
        if path and verbose:
            for node in path:
                print(node, end=" ")
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
        times.append(offset - onset)
    print("Average time:", sum(times) / n, "seconds")


if __name__ == '__main__':
    # test_n(1, True)
    test_n(1000)
