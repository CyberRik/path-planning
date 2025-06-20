
# Path Planning with RRT

## Overview

This project implements various variants of the Rapidly-Exploring Random Tree (RRT) algorithm for path planning in robotics. The basic RRT algorithm is enhanced with trajectory smoothing, spacing from obstacles, and a greedy approach that biases sampling towards the goal. The performance of these algorithms is evaluated through simulations.

## Features

- **Basic RRT**: A basic implementation of the RRT algorithm for path planning.
- **RRT with Spacing**: A variant of RRT that ensures the generated path maintains a safe distance from obstacles.
- **Greedy RRT**: A goal-biased variant of RRT that biases sampling towards the goal.
- **Trajectory Smoothing**: Applies smoothing to RRT paths to remove unnecessary intermediate nodes.

## Environment Setup

The environment is defined as a 100x100 grid with four fixed rectangular obstacles:

- **Obstacle 1**: Coordinates `((20, 20), (30, 30))`
- **Obstacle 2**: Coordinates `((50, 50), (60, 60))`
- **Obstacle 3**: Coordinates `((70, 20), (80, 30))`
- **Obstacle 4**: Coordinates `((20, 70), (30, 80))`

### Start and Goal Points
- **Start**: (10, 10)
- **Goal**: (90, 90)

## Installation

To run the code, make sure you have the following dependencies installed:

```bash
pip install -r requirements.txt
```

## Usage

### Running the Visualizer
To visualize the path planning algorithms, run the `app.py` script using Streamlit:

```bash
streamlit run app.py
```

This will launch a Streamlit web app where you can:
- Set the start and goal positions.
- Add obstacles and visualize the environment.
- Choose the RRT algorithm and view the path generated.
- Run benchmarking for the selected algorithm.

### Running Benchmarks
You can run performance benchmarks to evaluate the success rate, average path length, and planning time by setting the number of runs in the sidebar.

The benchmark results will be displayed in a table and can be downloaded as a CSV file.

## Files

- **app.py**: Streamlit app for visualizing the RRT algorithms and running benchmarks.
- **requirements.txt**: Required Python packages for the project.
- **visualizations.py**: Contains functions for visualizing the environment and RRT paths.
- **rrt_utils.py**: Defines utility functions and classes for RRT algorithms, such as the `Node` class and collision checking.
- **rrt_variants.py**: Implements the different variants of the RRT algorithm: basic RRT, RRT with spacing, and greedy RRT.
- **benchmark_results.csv**: CSV file containing the benchmark results.
- **path planning.pdf**: Report detailing the RRT algorithms and their evaluation.

## License

This project is licensed under the MIT License.

## Contact

Made with ❤️ by [CyberRik](https://github.com/CyberRik)
