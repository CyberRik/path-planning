import numpy as np
import matplotlib.pyplot as plt
import random

class Environment:
    def __init__(self, width, height, passage_width, wall_thickness, obstacle_width, spacing):
        self.width = width
        self.height = height
        self.passage_width = passage_width
        self.wall_thickness = wall_thickness
        self.obstacle_width = obstacle_width
        self.spacing = spacing

    def is_collision(self, point):
        # Check if point is inside the walls or obstacles
        obstacle_left = (self.width / 2 - self.passage_width / 2 - self.wall_thickness - self.obstacle_width, 0)
        obstacle_right = (self.width / 2 + self.passage_width / 2 + self.wall_thickness, 0)
        # Check for collision with left obstacle
        if obstacle_left[0] <= point[0] <= obstacle_left[0] + self.obstacle_width and \
           obstacle_left[1] <= point[1] <= self.height:
            return True
        # Check for collision with right obstacle
        if obstacle_right[0] <= point[0] <= obstacle_right[0] + self.obstacle_width and \
           obstacle_right[1] <= point[1] <= self.height:
            return True
        # Check if point is inside the walls
        if (self.width / 2 - self.passage_width / 2 - self.wall_thickness <= point[0] <= self.width / 2 + self.passage_width / 2 + self.wall_thickness) and \
           (0 <= point[1] <= self.height):
            return False  # No collision
        else:
            return True  # Collision

class Node:
    def __init__(self, point):
        self.point = point
        self.cost = 0.0
        self.parent = None

class RRT:
    def __init__(self, start, goal, environment, step_size=5, max_iter=1000):
        self.start = Node(start)
        self.goal = Node(goal)
        self.env = environment
        self.step_size = step_size
        self.max_iter = max_iter
        self.nodes = [self.start]

    def get_random_point(self):
        bias_factor = 2

        # Randomly select whether to generate a point near the edges of the obstacles or anywhere in the environment
        if random.random() < 0.5:
            # Generate random point near the edges of the obstacles
            obstacle_left = (self.env.width / 2 - self.env.passage_width / 2 - self.env.wall_thickness - self.env.obstacle_width, 0)
            obstacle_right = (self.env.width / 2 + self.env.passage_width / 2 + self.env.wall_thickness, 0)
            obstacle_edge_x = random.uniform(obstacle_left[0] + self.env.obstacle_width + self.env.spacing, obstacle_right[0] - self.env.spacing)
            obstacle_edge_y = random.uniform(self.env.spacing, self.env.height - self.env.spacing)
            return (obstacle_edge_x, obstacle_edge_y)
        else:
            # Generate random point anywhere in the environment
            return (random.uniform(0, self.env.width), random.uniform(0, self.env.height))

    def get_nearest_node(self, random_point):
        nearest_node = None
        min_dist = float('inf')
        for node in self.nodes:
            dist = np.linalg.norm(np.array(node.point) - np.array(random_point))
            if dist < min_dist:
                nearest_node = node
                min_dist = dist
        return nearest_node

    def steer(self, from_point, to_point):
        from_array = np.array(from_point)
        to_array = np.array(to_point)
        direction = to_array - from_array
        length = np.linalg.norm(direction)
        direction = direction / length
        new_point = from_array + direction * min(self.step_size, length)
        return tuple(new_point)

    def plan(self):
        for _ in range(self.max_iter):
            random_point = self.get_random_point()
            nearest_node = self.get_nearest_node(random_point)
            new_point = self.steer(nearest_node.point, random_point)

            if not self.env.is_collision(new_point):
                new_node = Node(new_point)
                new_node.parent = nearest_node
                new_node.cost = nearest_node.cost + np.linalg.norm(np.array(nearest_node.point) - np.array(new_point))
                self.nodes.append(new_node)

                if np.linalg.norm(np.array(new_node.point) - np.array(self.goal.point)) < self.step_size:
                    self.goal.parent = new_node
                    self.goal.cost = new_node.cost + np.linalg.norm(np.array(new_node.point) - np.array(self.goal.point))
                    self.nodes.append(self.goal)
                    return self.reconstruct_path()
        return None

    def reconstruct_path(self):
        path = []
        node = self.goal
        while node:
            path.append(node.point)
            node = node.parent
        return path[::-1]

# Define parameters for the environment
width = 50
height = 100
passage_width = 10
wall_thickness = 1
obstacle_width = 24  # Width of the obstacles
spacing = 2  # Spacing away from obstacles

# Define start and goal points
start = (width / 2, wall_thickness)
goal = (width / 2, height - wall_thickness)

# Create environment
env = Environment(width, height, passage_width, wall_thickness,obstacle_width,spacing)

# Planning with RRT
rrt = RRT(start, goal, env)
path = rrt.plan()

# Plotting
if path:
    path = np.array(path)
    plt.plot(path[:, 0], path[:, 1], '-o', label='Path')
plt.plot(start[0], start[1], 'go', label='Start')
plt.plot(goal[0], goal[1], 'ro', label='Goal')

# Plotting obstacles
obstacle_left = (width / 2 - passage_width / 2 - wall_thickness - obstacle_width, 0)
obstacle_right = (width / 2 + passage_width / 2 + wall_thickness, 0)
plt.fill([obstacle_left[0], obstacle_left[0] + obstacle_width, obstacle_left[0] + obstacle_width, obstacle_left[0]],
         [obstacle_left[1], obstacle_left[1], height, height], 'k')
plt.fill([obstacle_right[0], obstacle_right[0] + obstacle_width, obstacle_right[0] + obstacle_width, obstacle_right[0]],
         [obstacle_right[1], obstacle_right[1], height, height], 'k')

plt.xlim(0, width)
plt.ylim(0, height)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('RRT Path Planning in a Narrow Passage with Obstacles')
plt.legend()
plt.grid(True)
plt.show()

