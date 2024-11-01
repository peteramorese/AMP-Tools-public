import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import matplotlib.animation as animation
import math

visualize_config = {
    "default_agent_color": "dimgrey",
    "colliding_agent_color": "seagreen",
    "colliding_color": "red",
    "edge_lw": 3,
    "colliding_hatch": "x",
    "car_color": "powderblue",
    "goal_color": "mediumaquamarine"
}

def visualize_goal(initial_state : tuple, goal_state : list):
    ax = plt.gca()
    ax.scatter(initial_state[0], initial_state[1], c='slateblue', marker="*", label="Initial", s=25)
    vertices = [(goal_state[0][0], goal_state[1][1]), (goal_state[0][0], goal_state[1][0]), (goal_state[0][1], goal_state[1][0]), (goal_state[0][1], goal_state[1][1])]
    goal = Polygon(vertices, fill=True, facecolor=visualize_config["goal_color"] , edgecolor=visualize_config["goal_color"] , alpha=0.5, linewidth=visualize_config["edge_lw"])
    ax.add_patch(goal)

def visualize_agent(path : list, duration : list, length : float, width : float, animate : bool, isCar : bool):
    ax = plt.gca() 
    x_pts, y_pts, _ = zip(*path)
    
    times = [0]
    for dt in duration:
        times.append(dt + times[-1])

    isPoint = length == 0 or width == 0
    if animate:
        print("Saving animating to file_dump/video...")
        animate_point(path) if isPoint else animate_polygon(path, (length, width), isCar)
    elif not isPoint:
        plot_car_path(path, (length, width), isCar)

    ax.plot(x_pts, y_pts)
    ax.scatter(x_pts, y_pts, c=times, cmap='viridis', label="Time", s=15, zorder=2)

def plot_car_path(states : list, dimensions : tuple, isCar : bool):
    ax = plt.gca() 
    for state in states:
        patch = Polygon(get_vertices(state, dimensions, isCar), facecolor=visualize_config["car_color"] , edgecolor="black", lw=0.5)
        ax.add_patch(patch)

def animate_point(states):
    ax = plt.gca()
    point, = ax.plot([], [], marker='o', color="cadetblue") 

    def init():
        """Initialize the background of the animation."""
        point.set_data([], [])
        return point,

    def update(frame):
        """Update the point position for each frame."""
        point.set_data([states[frame][0]], [states[frame][1]])
        return point,

    duration = 10
    ani = animation.FuncAnimation(ax.get_figure(), update, frames=len(states), init_func=init, blit=True, repeat=False)
    ani.save('../../file_dump/video/path_animation.mp4', writer='ffmpeg', fps=len(states)/duration)
    # Uncomment the following line to display the animation
    # plt.show()

def animate_polygon(states, dimensions, isCar):
    ax = plt.gca()
    car_patch = Polygon(((0, 0), (0, 0), (0, 0), (0, 0)), facecolor=visualize_config["car_color"], edgecolor="black", lw=0.5)
    ax.add_patch(car_patch)

    def init():
        """Initialize the background of the animation."""
        car_patch.set_xy(((0, 0), (0, 0), (0, 0), (0, 0)))
        return car_patch,

    def update(frame):
        """Update the car position for each frame."""
        car_patch.set_xy(get_vertices(states[frame], dimensions, isCar))
        return car_patch,
    
    duration = 10
    ani = animation.FuncAnimation(ax.get_figure(), update, frames=len(states), init_func=init, blit=True, repeat=False)    
    ani.save('../../file_dump/video/path_animation.mp4', writer='ffmpeg', fps=len(states)/duration)
    # Uncomment the following line to display the animation
    # plt.show()


def get_vertices(state : tuple, dimensions : tuple, isCar : bool):
        half_l = dimensions[1] / 2
        half_w = dimensions[0] / 2
        ref_x = state[0] + half_w * math.cos(state[2]) if isCar else state[0]
        ref_y = state[1] + half_w * math.sin(state[2]) if isCar else state[1]
        vertices = [
            (ref_x + half_w * math.cos(state[2]) - half_l * math.sin(state[2]),
            ref_y + half_w * math.sin(state[2]) + half_l * math.cos(state[2])),

            (ref_x - half_w * math.cos(state[2]) - half_l * math.sin(state[2]),
            ref_y - half_w * math.sin(state[2]) + half_l * math.cos(state[2])),

            (ref_x - half_w * math.cos(state[2]) + half_l * math.sin(state[2]),
            ref_y - half_w * math.sin(state[2]) - half_l * math.cos(state[2])),

            (ref_x + half_w * math.cos(state[2]) + half_l * math.sin(state[2]),
            ref_y + half_w * math.sin(state[2]) - half_l * math.cos(state[2]))
        ]
        return vertices