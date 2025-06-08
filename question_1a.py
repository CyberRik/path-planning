import random
import math
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

def steer(from_node, to_point, max_distance):
    angle = math.atan2(to_point[1] - from_node.y, to_point[0] - from_node.x)
    new_x = from_node.x + max_distance * math.cos(angle)
    new_y = from_node.y + max_distance * math.sin(angle)
    return Node(new_x, new_y)

def is_collision_free(node1, node2, obstacles):
    for (ox1, oy1), (ox2, oy2) in obstacles:
        if line_intersects_rect(node1.x, node1.y, node2.x, node2.y, ox1, oy1, ox2, oy2):
            return False
    return True

def get_random_point(width, height):
    return (random.uniform(0, width), random.uniform(0, height))

def get_nearest_node(nodes, point):
    nearest_node = nodes[0]
    min_dist = distance(nearest_node, Node(point[0], point[1]))
    for node in nodes:
        dist = distance(node, Node(point[0], point[1]))
        if dist < min_dist:
            nearest_node = node
            min_dist = dist
    return nearest_node

def line_intersects_rect(x1, y1, x2, y2, rx1, ry1, rx2, ry2):
    INSIDE, LEFT, RIGHT, BOTTOM, TOP = 0, 1, 2, 4, 8
    def compute_out_code(x, y):
        code = INSIDE
        if x < rx1: code |= LEFT
        elif x > rx2: code |= RIGHT
        if y < ry1: code |= BOTTOM
        elif y > ry2: code |= TOP
        return code

    outcode1 = compute_out_code(x1, y1)
    outcode2 = compute_out_code(x2, y2)
    while True:
        if not (outcode1 | outcode2):
            return True
        elif outcode1 & outcode2:
            return False
        else:
            x, y = 0, 0
            outcode_out = outcode1 if outcode1 else outcode2
            if outcode_out & TOP:
                x = x1 + (x2 - x1) * (ry2 - y1) / (y2 - y1)
                y = ry2
            elif outcode_out & BOTTOM:
                x = x1 + (x2 - x1) * (ry1 - y1) / (y2 - y1)
                y = ry1
            elif outcode_out & RIGHT:
                y = y1 + (y2 - y1) * (rx2 - x1) / (x2 - x1)
                x = rx2
            elif outcode_out & LEFT:
                y = y1 + (y2 - y1) * (rx1 - x1) / (x2 - x1)
                x = rx1
            if outcode_out == outcode1:
                x1, y1 = x, y
                outcode1 = compute_out_code(x1, y1)
            else:
                x2, y2 = x, y
                outcode2 = compute_out_code(x2, y2)

def plot_environment(start, goal, obstacles, path=None, title="Environment"):
    plt.figure()
    plt.title(title)

    # Plot obstacles
    for (ox1, oy1), (ox2, oy2) in obstacles:
        plt.gca().add_patch(plt.Rectangle((ox1, oy1), ox2 - ox1, oy2 - oy1, fc='blue'))

    # Plot start and goal points
    plt.plot(start[0], start[1], 'go', markersize=10, label='Start')
    plt.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')

    # Plot path if available
    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_x, path_y, 'b-', linewidth=2, label='Path')
        plt.plot(path_x, path_y, 'bo', markersize=5)

    plt.xlim(0, 100)
    plt.ylim(0, 100)
    plt.legend()
    plt.grid(True)
    plt.show()

def rrt(start, goal, obstacles, width, height, max_distance, goal_tolerance):
    start_node = Node(start[0], start[1])
    goal_node = Node(goal[0], goal[1])
    nodes = [start_node]

    while True:
        random_point = get_random_point(width, height)
        nearest_node = get_nearest_node(nodes, random_point)
        theta = math.atan2(random_point[1] - nearest_node.y, random_point[0] - nearest_node.x)
        new_x = nearest_node.x + max_distance * math.cos(theta)
        new_y = nearest_node.y + max_distance * math.sin(theta)
        new_node = Node(new_x, new_y)

        if not is_collision_free(nearest_node, new_node, obstacles):
            continue

        new_node.parent = nearest_node
        nodes.append(new_node)

        if distance(new_node, goal_node) < goal_tolerance:
            goal_node.parent = new_node
            nodes.append(goal_node)
            break

    path = []
    node = goal_node
    while node is not None:
        path.append((node.x, node.y))
        node = node.parent
    path.reverse()

    return path

if __name__ == "__main__":
    # Define the start, goal, and environment boundaries
    start = (10, 10)
    goal = (90, 90)
    width, height = 100, 100
    max_distance = 10  # Maximum distance between nodes
    goal_tolerance = 5  # Distance within which the goal is considered reached

    # Define the obstacles
    obstacles = [
        ((20, 20), (30, 30)),
        ((50, 50), (60, 60)),
        ((70, 20), (80, 30)),
        ((20, 70), (30, 80))
    ]
    print("Generating path...")
    path = rrt(start, goal, obstacles, width, height, max_distance, goal_tolerance)
    plot_environment(start, goal, obstacles, path)

