import os
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.colors import Normalize, LinearSegmentedColormap
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
    ax.scatter(initial_state[0], initial_state[1], c='blue', label="Initial", s=15)
    vertices = [(goal_state[0][0], goal_state[1][1]), (goal_state[0][0], goal_state[1][0]), (goal_state[0][1], goal_state[1][0]), (goal_state[0][1], goal_state[1][1])]
    goal = Polygon(vertices, fill=True, facecolor=visualize_config["goal_color"] , edgecolor=visualize_config["goal_color"] , alpha=0.5, linewidth=visualize_config["edge_lw"])
    ax.add_patch(goal)

def visualize_car(path : list, duration : list, length : float = 0, width : float = 0, shift : bool = True):
    ax = plt.gca() 
    x_pts = list()
    y_pts = list()
    t_pts = list()
    for waypt in path:
        x_pts.append(waypt[0])
        y_pts.append(waypt[1])
        t_pts.append(waypt[2])
    
    times = [0]
    for dt in duration:
        times.append(dt + times[-1])

    states = list(zip(x_pts, y_pts, t_pts))
    if length and width is not None:
        plot_car_path(states, (length, width), shift)
    ax.plot(x_pts, y_pts)
    ax.scatter(x_pts, y_pts, c=times, cmap='viridis', label="Time", s=15, zorder=2)

def plot_car_path(states : list, dimensions : tuple, isPoint : bool = True):
    ax = plt.gca() 
    half_l = dimensions[1] / 2
    half_w = dimensions[0] / 2
    for state in states:
        center_x = state[0] + half_w * math.cos(state[2]) if not isPoint else state[0]
        center_y = state[1] + half_w * math.sin(state[2]) if not isPoint else state[1]
        vertices = [
            (center_x + half_w * math.cos(state[2]) - half_l * math.sin(state[2]),
            center_y + half_w * math.sin(state[2]) + half_l * math.cos(state[2])),

            (center_x - half_w * math.cos(state[2]) - half_l * math.sin(state[2]),
            center_y - half_w * math.sin(state[2]) + half_l * math.cos(state[2])),

            (center_x - half_w * math.cos(state[2]) + half_l * math.sin(state[2]),
            center_y - half_w * math.sin(state[2]) - half_l * math.cos(state[2])),

            (center_x + half_w * math.cos(state[2]) + half_l * math.sin(state[2]),
            center_y + half_w * math.sin(state[2]) - half_l * math.cos(state[2]))
        ]
        patch = Polygon(vertices, facecolor=visualize_config["car_color"] , edgecolor="black", lw=0.5)
        ax.add_patch(patch)