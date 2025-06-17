# visualizations.py
import matplotlib.pyplot as plt

def plot_environment(start, goal, obstacles, path=None, title="Environment"):
    plt.figure()
    plt.title(title)

    for (ox1, oy1), (ox2, oy2) in obstacles:
        plt.gca().add_patch(plt.Rectangle((ox1, oy1), ox2 - ox1, oy2 - oy1, fc='blue'))

    plt.plot(start[0], start[1], 'go', markersize=10, label='Start')
    plt.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')

    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_x, path_y, 'b-', linewidth=2, label='Path')
        plt.plot(path_x, path_y, 'bo', markersize=5)

    plt.xlim(0, 100)
    plt.ylim(0, 100)
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_comparison_path(start, goal, obstacles, unsmoothed_path, smoothed_path=None, title="Path Comparison"):
    plt.figure()
    plt.title(title)

    # Plot obstacles
    for (ox1, oy1), (ox2, oy2) in obstacles:
        plt.gca().add_patch(plt.Rectangle((ox1, oy1), ox2 - ox1, oy2 - oy1, fc='blue'))

    plt.plot(start[0], start[1], 'go', markersize=10, label='Start')
    plt.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')

    if unsmoothed_path:
        path_x, path_y = zip(*unsmoothed_path)
        plt.plot(path_x, path_y, 'b-', linewidth=2, label='Unsmoothed Path')

    if smoothed_path:
        path_x, path_y = zip(*smoothed_path)
        plt.plot(path_x, path_y, 'r--', linewidth=2, label='Smoothed Path')

    plt.xlim(0, 100)
    plt.ylim(0, 100)
    plt.legend()
    plt.grid(True)
    plt.show()
