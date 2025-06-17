# rrt_variants.py
import random
import numpy as np
from rrt_utils import Node, Environment, distance, steer

# Basic RRT
def rrt_basic(start, goal, environment, max_distance=10, goal_tolerance=5):
    start_node = Node(start[0], start[1])
    goal_node = Node(goal[0], goal[1])
    nodes = [start_node]
    
    while True:
        random_point = (random.uniform(0, environment.width), random.uniform(0, environment.height))
        nearest_node = min(nodes, key=lambda node: distance(node, Node(random_point[0], random_point[1])))
        new_node = steer(nearest_node, random_point, max_distance)
        
        if environment.is_collision_free(nearest_node, new_node):
            new_node.parent = nearest_node
            nodes.append(new_node)
            if distance(new_node, goal_node) < goal_tolerance:
                goal_node.parent = new_node
                break
    path = []
    node = goal_node
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

# RRT with Spacing (to avoid obstacles more smartly)
def rrt_with_spacing(start, goal, environment, max_distance=10, goal_tolerance=5, spacing=5):
    start_node = Node(start[0], start[1])
    goal_node = Node(goal[0], goal[1])
    nodes = [start_node]

    def is_point_valid(point):
        for (ox1, oy1), (ox2, oy2) in environment.obstacles:
            if ox1 - spacing <= point[0] <= ox2 + spacing and oy1 - spacing <= point[1] <= oy2 + spacing:
                return False
        return True

    while True:
        random_point = (random.uniform(0, environment.width), random.uniform(0, environment.height))
        if not is_point_valid(random_point):
            continue

        nearest_node = min(nodes, key=lambda node: distance(node, Node(random_point[0], random_point[1])))
        new_node = steer(nearest_node, random_point, max_distance)

        if environment.is_collision_free(nearest_node, new_node):
            new_node.parent = nearest_node
            nodes.append(new_node)
            if distance(new_node, goal_node) < goal_tolerance:
                goal_node.parent = new_node
                break
    path = []
    node = goal_node
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

# Greedy RRT (Goal-biased)
def rrt_greedy(start, goal, environment, max_distance=10, goal_tolerance=5, goal_bias=0.2):
    start_node = Node(start[0], start[1])
    goal_node = Node(goal[0], goal[1])
    nodes = [start_node]

    def biased_random_point():
        if random.random() < goal_bias:
            return goal
        return (random.uniform(0, environment.width), random.uniform(0, environment.height))

    while True:
        random_point = biased_random_point()
        nearest_node = min(nodes, key=lambda node: distance(node, Node(random_point[0], random_point[1])))
        new_node = steer(nearest_node, random_point, max_distance)

        if environment.is_collision_free(nearest_node, new_node):
            new_node.parent = nearest_node
            nodes.append(new_node)
            if distance(new_node, goal_node) < goal_tolerance:
                goal_node.parent = new_node
                break
    path = []
    node = goal_node
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

# Smoothed RRT (Optional for path smoothing)
def smooth_path(path, environment):
    smoothed_path = [path[0]]
    i = 0
    while i < len(path) - 1:
        for j in range(len(path) - 1, i, -1):
            if environment.is_collision_free(Node(path[i][0], path[i][1]), Node(path[j][0], path[j][1])):
                smoothed_path.append(path[j])
                i = j
                break
    return smoothed_path
