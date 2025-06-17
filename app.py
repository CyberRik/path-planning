import random
import matplotlib.pyplot as plt
import streamlit as st
import numpy as np
import pandas as pd
import time
from rrt_utils import Environment
from rrt_variants import rrt_basic, rrt_with_spacing, rrt_greedy, smooth_path
from visualizations import plot_environment, plot_comparison_path
from streamlit_drawable_canvas import st_canvas

width, height = 100, 100

def run_benchmark(rrt_function, env, num_runs=1000, smooth=False, **kwargs):
    success_count = 0
    total_length = 0
    total_time = 0
    node_counts = []

    for _ in range(num_runs):
        start_time = time.time()
        path = rrt_function(start, goal, env, **kwargs)
        total_time += time.time() - start_time

        if path:
            success_count += 1
            total_length += len(path)
            node_counts.append(len(path))

        if smooth:
            path = smooth_path(path, env)

    success_rate = success_count / num_runs
    avg_length = total_length / success_count if success_count > 0 else 0
    avg_time = total_time / num_runs

    return success_rate, avg_length, avg_time, node_counts


st.title("RRT Algorithm Visualizer and Benchmarking")

# Sidebar for environment setup
st.sidebar.header("Environment Setup")

start = st.sidebar.text_input("Start position (x, y)", "(10, 10)")
goal = st.sidebar.text_input("Goal position (x, y)", "(90, 90)")

start = tuple(map(int, start.strip("()").split(",")))
goal = tuple(map(int, goal.strip("()").split(",")))

# Slider for obstacle size
obstacle_size = st.sidebar.slider("Obstacle Size", min_value=5, max_value=30, value=10)

# Add input for obstacles (allow user to input only the bottom-left corner)
st.sidebar.subheader("Add Obstacles")
obstacles_input = st.sidebar.text_area("Enter bottom-left point of obstacles (e.g., [(x1, y1), ...])", 
                                      "[((20, 20)), ((50, 50)), ((70, 20)), ((20, 70))]")

try:
    bottom_left_points = eval(obstacles_input)  # Convert the input string to a list of tuples
    obstacles = [((x1, y1), (x1 + obstacle_size, y1 + obstacle_size)) for (x1, y1) in bottom_left_points]
except:
    obstacles = []

# Set the environment with the user-defined obstacles
env = Environment(100, 100, obstacles)

# Default plot of environment with obstacles
st.subheader("Current Environment with Obstacles")
fig, ax = plt.subplots()
plot_environment(start, goal, obstacles, ax=ax, title="Environment with Movable Obstacles")
st.pyplot(fig)

# Dropdown for selecting the algorithm
algorithm = st.sidebar.selectbox("Choose RRT Algorithm", ["Basic RRT", "Spacing-aware RRT", "Greedy RRT"])
st.sidebar.write(f"Selected Algorithm: {algorithm}")



# Button to plot the selected algorithm's path
if st.sidebar.button("Plot Algorithm Path"):
    # Update the environment and run the selected algorithm
    if algorithm == "Basic RRT":
        unsmoothed_path = rrt_basic(start, goal, env)
        smoothed_path = smooth_path(unsmoothed_path, env)
        st.subheader("Basic RRT Path")
        fig, ax = plt.subplots()
        plot_comparison_path(start, goal, obstacles, unsmoothed_path, smoothed_path, ax=ax, title="Basic RRT Path")
        st.pyplot(fig)

    elif algorithm == "Spacing-aware RRT":
        path_spacing = rrt_with_spacing(start, goal, env, spacing=5)
        st.subheader("Spacing-aware RRT Path")
        fig, ax = plt.subplots()
        plot_environment(start, goal, obstacles, path_spacing, ax=ax, title="Spacing-aware RRT Path")
        st.pyplot(fig)

    elif algorithm == "Greedy RRT":
        unsmoothed_path_greedy = rrt_greedy(start, goal, env, goal_bias=0.2)
        smoothed_path_greedy = smooth_path(unsmoothed_path_greedy, env)
        st.subheader("Greedy RRT Path")
        fig, ax = plt.subplots()
        plot_comparison_path(start, goal, obstacles, unsmoothed_path_greedy, smoothed_path_greedy, ax=ax, title="Greedy RRT Path")
        st.pyplot(fig)

# Slider for the number of benchmark runs before the button
num_runs = st.sidebar.slider("Number of Runs", min_value=10, max_value=1000, value=100)

# Button to run the benchmark
if st.sidebar.button("Run Benchmark"):
    success_rate, avg_length, avg_time, node_counts = run_benchmark(rrt_basic, env, num_runs)
    
    st.subheader("Benchmark Results")
    df = pd.DataFrame({
        'Algorithm': ['Basic RRT'],
        'Success Rate': [success_rate],
        'Average Path Length': [avg_length],
        'Average Planning Time': [avg_time],
        'Average Node Count': [np.mean(node_counts)]
    })

    st.write(df)

    df.to_csv("benchmark_results.csv", index=False)
