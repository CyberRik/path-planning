# rrt_utils.py
import numpy as np
import math

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

class Environment:
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.obstacles = obstacles

    def is_collision_free(self, node1, node2):
        for (ox1, oy1), (ox2, oy2) in self.obstacles:
            if line_intersects_rect(node1.x, node1.y, node2.x, node2.y, ox1, oy1, ox2, oy2):
                return False
        return True

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
