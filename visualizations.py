import matplotlib.pyplot as plt

def plot_environment(start, goal, obstacles, path=None, title="Environment", ax=None):
    if ax is None:
        fig, ax = plt.subplots()  

    ax.set_title(title)

    for (ox1, oy1), (ox2, oy2) in obstacles:
        ax.add_patch(plt.Rectangle((ox1, oy1), ox2 - ox1, oy2 - oy1, fc='blue'))

    ax.plot(start[0], start[1], 'go', markersize=10, label='Start')
    ax.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')

    if path:
        path_x, path_y = zip(*path)
        ax.plot(path_x, path_y, 'b-', linewidth=2, label='Path')
        ax.plot(path_x, path_y, 'bo', markersize=5)

    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.legend()
    ax.grid(True)

def plot_comparison_path(start, goal, obstacles, unsmoothed_path, smoothed_path=None, title="Path Comparison", ax=None):
    if ax is None:
        fig, ax = plt.subplots() 

    ax.set_title(title)

    for (ox1, oy1), (ox2, oy2) in obstacles:
        ax.add_patch(plt.Rectangle((ox1, oy1), ox2 - ox1, oy2 - oy1, fc='blue'))

    ax.plot(start[0], start[1], 'go', markersize=10, label='Start')
    ax.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')

    if unsmoothed_path:
        path_x, path_y = zip(*unsmoothed_path)
        ax.plot(path_x, path_y, 'b-', linewidth=2, label='Unsmoothed Path')

    if smoothed_path:
        path_x, path_y = zip(*smoothed_path)
        ax.plot(path_x, path_y, 'r--', linewidth=2, label='Smoothed Path')

    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.legend()
    ax.grid(True)
