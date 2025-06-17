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

#sidebar
st.sidebar.header("Environment Setup")

start = st.sidebar.text_input("Start position (x, y)", "(10, 10)")
goal = st.sidebar.text_input("Goal position (x, y)", "(90, 90)")

start = tuple(map(int, start.strip("()").split(",")))
goal = tuple(map(int, goal.strip("()").split(",")))

initial_obstacles = [
    ((20, 20), (30, 30)),
    ((50, 50), (60, 60)),
    ((70, 20), (80, 30)),
    ((20, 70), (30, 80))
]

# def update_obstacles_from_canvas(canvas_result):
#     obstacles = []
#     for shape in canvas_result.objects:
#         if shape['type'] == 'rect':
#             ox1, oy1 = shape['left'], shape['top']
#             ox2, oy2 = ox1 + shape['width'], oy1 + shape['height']
#             obstacles.append(((ox1, oy1), (ox2, oy2)))
#     return obstacles

# canvas = st_canvas(
#     width=500,
#     height=500,
#     stroke_width=2,
#     stroke_color="blue",
#     background_color="white",
#     drawing_mode="rect",
#     initial_drawing_objects=[{
#         'type': 'rect', 'left': ox1, 'top': oy1, 'width': ox2 - ox1, 'height': oy2 - oy1, 
#         'stroke': 'blue', 'fill': 'rgba(0, 0, 255, 0.3)', 'key': f'obj_{i}'
#     } for i, ((ox1, oy1), (ox2, oy2)) in enumerate(initial_obstacles)],
#     key="canvas"
# )


updated_obstacles = initial_obstacles 

env = Environment(100, 100, updated_obstacles)
st.subheader("Current Environment with Obstacles")
fig, ax = plt.subplots()
plot_environment(start, goal, updated_obstacles, ax=ax, title="Environment with Movable Obstacles")
st.pyplot(fig)  

algorithm = st.sidebar.selectbox("Choose RRT Algorithm", ["Basic RRT", "Spacing-aware RRT", "Greedy RRT"])
st.sidebar.write(f"Selected Algorithm: {algorithm}")

if algorithm == "Basic RRT":
    unsmoothed_path = rrt_basic(start, goal, env)
    smoothed_path = smooth_path(unsmoothed_path, env)
    st.subheader("Basic RRT Path")
    fig, ax = plt.subplots()
    plot_comparison_path(start, goal, updated_obstacles, unsmoothed_path, smoothed_path, ax=ax, title="Basic RRT Path")
    st.pyplot(fig)

elif algorithm == "Spacing-aware RRT":
    path_spacing = rrt_with_spacing(start, goal, env, spacing=5)
    st.subheader("Spacing-aware RRT Path")
    fig, ax = plt.subplots()
    plot_environment(start, goal, updated_obstacles, path_spacing, ax=ax, title="Spacing-aware RRT Path")
    st.pyplot(fig)

elif algorithm == "Greedy RRT":
    unsmoothed_path_greedy = rrt_greedy(start, goal, env, goal_bias=0.2)
    smoothed_path_greedy = smooth_path(unsmoothed_path_greedy, env)
    st.subheader("Greedy RRT Path")
    fig, ax = plt.subplots()
    plot_comparison_path(start, goal, updated_obstacles, unsmoothed_path_greedy, smoothed_path_greedy, ax=ax, title="Spacing-aware RRT Path")
    st.pyplot(fig)

if st.sidebar.button("Run Benchmark"):
    num_runs = st.sidebar.slider("Number of Runs", min_value=10, max_value=1000, value=100)
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
