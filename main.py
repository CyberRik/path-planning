# main.py
import random
import matplotlib.pyplot as plt
from rrt_variants import rrt_basic, rrt_with_spacing, rrt_greedy, smooth_path
from visualizations import plot_environment
from rrt_utils import Environment

# Define environment parameters
width, height = 100, 100
obstacles = [
    ((20, 20), (30, 30)),
    ((50, 50), (60, 60)),
    ((70, 20), (80, 30)),
    ((20, 70), (30, 80))
]
start = (10, 10)
goal = (90, 90)

# Create environment object
env = Environment(width, height, obstacles)

# Run Basic RRT
path_basic = rrt_basic(start, goal, env)
plot_environment(start, goal, obstacles, path_basic, "Basic RRT Path")

# Run Spacing-aware RRT (with additional spacing parameter)
path_spacing = rrt_with_spacing(start, goal, env, spacing=5)
plot_environment(start, goal, obstacles, path_spacing, "Spacing-aware RRT Path")

# Run Greedy RRT (with goal bias parameter)
path_greedy = rrt_greedy(start, goal, env, goal_bias=0.2)
plot_environment(start, goal, obstacles, path_greedy, "Greedy RRT Path")

# Optional: Run Smoothed Path (for Basic RRT example)
path_smoothed = smooth_path(path_basic, env)
plot_environment(start, goal, obstacles, path_smoothed, "Smoothed Basic RRT Path")
