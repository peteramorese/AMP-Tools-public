import os
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.colors import Normalize, LinearSegmentedColormap
import numpy as np
import sys

visualize_config = {
    "default_agent_color": "dimgrey",
    "colliding_agent_color": "red",
    "colliding_color": "red",
    "edge_lw": 3,
    "colliding_hatch": "x"
}

def visualize_circle_agent(radius : float, centerpoint : tuple, colliding : bool, cmap_scale = None, agent_color = None):
    ax = plt.gca() 

    if cmap_scale is None:
        color = visualize_config["default_agent_color"] if not colliding else visualize_config["colliding_agent_color"]
    else:
        color_map = LinearSegmentedColormap.from_list('custom', ['indianred', 'seagreen'], N=256)
        scalar_map = mpl.cm.ScalarMappable(norm=Normalize(vmin=0, vmax=1), cmap=color_map)
        color = scalar_map.to_rgba(cmap_scale)
    alpha = 0.3 if colliding else 1.0
    facecolor = agent_color if agent_color is not None else color
    hatch = visualize_config["colliding_hatch"] if colliding else None
    patch = Circle(centerpoint, radius=radius, fill=True, facecolor=facecolor, edgecolor=color, alpha=alpha, linewidth=visualize_config["edge_lw"], hatch=hatch)
    ax.add_patch(patch)

def visualize_path(radius : float, path : list, random_color = False, collision_states = None):
    ax = plt.gca() 
    x_pts = list()
    y_pts = list()
    x_col_pts = list()
    y_col_pts = list()
    for waypt in path:
        x_pts.append(waypt[0])
        y_pts.append(waypt[1])
    
    scale = 0.0
    agent_color = np.random.rand(3) if random_color else None
    for state in path:
        visualize_circle_agent(radius, state, False, scale, agent_color)
        scale += 1.0 / len(path)
    
    if collision_states is not None:
        for state in collision_states:
            visualize_circle_agent(radius, state, True, None, agent_color)
