# visualizations.py
import matplotlib.pyplot as plt

def plot_environment(start, goal, obstacles, path=None, title="Environment"):
    plt.figure()
    plt.title(title)

    # Plot obstacles
    for (ox1, oy1), (ox2, oy2) in obstacles:
        plt.gca().add_patch(plt.Rectangle((ox1, oy1), ox2 - ox1, oy2 - oy1, fc='blue'))

    # Plot start and goal points
    plt.plot(start[0], start[1], 'go', markersize=10, label='Start')
    plt.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')

    # Plot path if available
    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_x, path_y, 'b-', linewidth=2, label='Path')
        plt.plot(path_x, path_y, 'bo', markersize=5)

    plt.xlim(0, 100)
    plt.ylim(0, 100)
    plt.legend()
    plt.grid(True)
    plt.show()
