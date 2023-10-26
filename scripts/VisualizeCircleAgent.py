import os
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.colors import Normalize, LinearSegmentedColormap
import numpy as np
import sys
#from VisualizeEnvironment import Environment2DVisualizer

visualize_config = {
    "default_agent_color": "dimgrey",
    "colliding_color": "red",
}

def visualize_circle_agent(radius : float, centerpoint : tuple, colliding : bool, cmap_scale = None, auto_bound_axes = True):
    ax = plt.gca() 

    if cmap_scale is None:
        color = visualize_config["link_color"] if not colliding else visualize_config["colliding_link_color"]
    else:
        color_map = LinearSegmentedColormap.from_list('custom', ['indianred', 'seagreen'], N=256)
        scalar_map = mpl.cm.ScalarMappable(norm=Normalize(vmin=0, vmax=1), cmap=color_map)
        color = scalar_map.to_rgba(cmap_scale)
    alpha = 0.3 if colliding else 1.0
    patch = Circle(centerpoint, radius=radius, fill=True, facecolor=color, alhpa=alpha)
    ax.add_patch(patch)
