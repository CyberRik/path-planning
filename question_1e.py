import numpy as np
import matplotlib.pyplot as plt
from question_1a import rrt as rrt_basic
from question_1b import smooth_path
from question_1c import rrt_with_spacing
from question_1d import rrt_greedy

# Define the start, goal, and environment boundaries
start = (10, 10)
goal = (90, 90)
width, height = 100, 100
max_distance = 10  # Maximum distance between nodes
goal_tolerance = 5  # Distance within which the goal is considered reached
goal_bias = 0.2  # Bias towards the goal
spacing = 5  # Minimum distance from obstacles

# Define the obstacles
obstacles = [
    ((20, 20), (30, 30)),
    ((50, 50), (60, 60)),
    ((70, 20), (80, 30)),
    ((20, 70), (30, 80))
]

def run_experiment(rrt_function, num_runs=1000, smooth=False, **kwargs):
    node_counts = []
    for _ in range(num_runs):
        result = rrt_function(start, goal, obstacles, width, height, max_distance, goal_tolerance, **kwargs)
        if isinstance(result, tuple):
            path, nodes = result
        else:
            path = result

        if smooth:
            path = smooth_path(path, obstacles)
        node_counts.append(len(path))
    return np.mean(node_counts)

# Run experiments
num_runs = 1000
avg_nodes_basic = run_experiment(rrt_basic, num_runs)
avg_nodes_smoothed = run_experiment(rrt_basic, num_runs, smooth=True)
avg_nodes_spacing = run_experiment(rrt_with_spacing, num_runs, spacing=spacing)
avg_nodes_greedy = run_experiment(rrt_greedy, num_runs, goal_bias=goal_bias)

# Print results (optional)
print(f"Average number of nodes (Basic RRT): {avg_nodes_basic}")
print(f"Average number of nodes (Smoothed Path): {avg_nodes_smoothed}")
print(f"Average number of nodes (RRT with Spacing): {avg_nodes_spacing}")
print(f"Average number of nodes (RRT with Greedy Approach): {avg_nodes_greedy}")

# Plot results
methods = ['Basic RRT', 'Smoothed Path', 'RRT with Spacing', 'RRT with Greedy Approach']
averages = [avg_nodes_basic, avg_nodes_smoothed, avg_nodes_spacing, avg_nodes_greedy]

plt.figure(figsize=(10, 6))
plt.bar(methods, averages, color=['blue', 'orange', 'green', 'red'])
plt.xlabel('RRT Method')
plt.ylabel('Average Number of Nodes')
plt.title('Average Number of Nodes for Different RRT Methods')
plt.show()

