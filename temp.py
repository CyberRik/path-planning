# main.py
import random
import matplotlib.pyplot as plt
from rrt_variants import rrt_basic, rrt_with_spacing, rrt_greedy, smooth_path
from visualizations import plot_environment, plot_comparison_path
from rrt_utils import Environment
import pandas as pd
import time

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
unsmoothed_path_basic = rrt_basic(start, goal, env)
smoothed_path_basic = smooth_path(unsmoothed_path_basic, env)

# Plot Basic RRT (unsmoothed vs smoothed path)
plt.figure()  # Create a new figure
plot_comparison_path(start, goal, obstacles, unsmoothed_path_basic, smoothed_path_basic, title="Basic RRT: Smoothed vs Unsmoothed Path")
plt.close()  # Close the figure after showing it

# Run Spacing-aware RRT (with additional spacing parameter)
path_spacing = rrt_with_spacing(start, goal, env, spacing=5)
plt.figure()  # Create a new figure
plot_environment(start, goal, obstacles, path_spacing, "Spacing-aware RRT Path")
plt.close()  # Close the figure after showing it

# Run Greedy RRT (unsmoothed)
unsmoothed_path_greedy = rrt_greedy(start, goal, env, goal_bias=0.2)

# Run Path Smoothing (smoothed)
smoothed_path_greedy = smooth_path(unsmoothed_path_greedy, env)

# Plot Greedy RRT (unsmoothed vs smoothed path)
plt.figure()  # Create a new figure
plot_comparison_path(start, goal, obstacles, unsmoothed_path_greedy, smoothed_path_greedy, title="Greedy RRT: Smoothed vs Unsmoothed Path")
plt.close()  # Close the figure after showing it

# Function to run the benchmark
def run_benchmark(rrt_function, environment, num_runs=1000, smooth=False, **kwargs):
    success_count = 0
    total_length = 0
    total_time = 0
    node_counts = []
    
    for _ in range(num_runs):
        start_time = time.time()
        path = rrt_function(start, goal, environment, **kwargs)
        total_time += time.time() - start_time

        if path:
            success_count += 1
            total_length += len(path)
            node_counts.append(len(path))
        
        if smooth:
            path = smooth_path(path, environment)
    
    # Calculate averages
    success_rate = success_count / num_runs
    avg_length = total_length / success_count if success_count > 0 else 0
    avg_time = total_time / num_runs

    return success_rate, avg_length, avg_time, node_counts

# Run benchmarks for all variants
num_runs = 1000
env = Environment(100, 100, obstacles)

# Run Basic RRT benchmark
success_rate_basic, avg_length_basic, avg_time_basic, node_counts_basic = run_benchmark(rrt_basic, env, num_runs)

# Run Spacing-aware RRT benchmark
success_rate_spacing, avg_length_spacing, avg_time_spacing, node_counts_spacing = run_benchmark(rrt_with_spacing, env, num_runs, spacing=5)

# Run Greedy RRT benchmark
success_rate_greedy, avg_length_greedy, avg_time_greedy, node_counts_greedy = run_benchmark(rrt_greedy, env, num_runs, goal_bias=0.2)

# Store results in DataFrame
df = pd.DataFrame({
    'Algorithm': ['Basic RRT', 'Spacing-aware RRT', 'Greedy RRT'],
    'Success Rate': [success_rate_basic, success_rate_spacing, success_rate_greedy],
    'Average Path Length': [avg_length_basic, avg_length_spacing, avg_length_greedy],
    'Average Planning Time': [avg_time_basic, avg_time_spacing, avg_time_greedy],
    'Average Node Count': [
        sum(node_counts_basic) / len(node_counts_basic),
        sum(node_counts_spacing) / len(node_counts_spacing),
        sum(node_counts_greedy) / len(node_counts_greedy)
    ]
})

# Save results to CSV
df.to_csv('benchmark_results.csv', index=False)

# Plot Benchmarking Results
# Success Rate Bar Chart
plt.figure()  # Create a new figure
df.plot(kind='bar', x='Algorithm', y='Success Rate', color=['blue', 'orange', 'green'], legend=False)
plt.title('Success Rate Comparison')
plt.ylabel('Success Rate')
plt.show()

# Path Length Boxplot
plt.figure()  # Create a new figure
df.boxplot(column='Average Path Length', by='Algorithm')
plt.title('Path Length Comparison')
plt.ylabel('Average Path Length')
plt.show()

# Planning Time Boxplot
plt.figure()  # Create a new figure
df.boxplot(column='Average Planning Time', by='Algorithm')
plt.title('Planning Time Comparison')
plt.ylabel('Average Planning Time')
plt.show()

# Display all plots at once
plt.show()
