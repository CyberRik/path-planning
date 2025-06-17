# main.py
import random
import matplotlib.pyplot as plt
from rrt_variants import rrt_basic, rrt_with_spacing, rrt_greedy, smooth_path
from visualizations import plot_environment, plot_comparison_path
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
unsmoothed_path = rrt_basic(start, goal, env)
smoothed_path = smooth_path(unsmoothed_path, env)

# Plot both paths
plot_comparison_path(start, goal, obstacles, unsmoothed_path, smoothed_path, title="Smoothed vs Unsmoothed Path")

# Run Spacing-aware RRT (with additional spacing parameter)
path_spacing = rrt_with_spacing(start, goal, env, spacing=5)
plot_environment(start, goal, obstacles, path_spacing, "Spacing-aware RRT Path")

# Run Greedy RRT (unsmoothed)
unsmoothed_path_greedy = rrt_greedy(start, goal, env, goal_bias=0.2)

# Run Path Smoothing (smoothed)
smoothed_path_greedy = smooth_path(unsmoothed_path_greedy, env)

# Plot both paths
plot_comparison_path(start, goal, obstacles, unsmoothed_path_greedy, smoothed_path_greedy, title="Greedy RRT: Smoothed vs Unsmoothed Path")

