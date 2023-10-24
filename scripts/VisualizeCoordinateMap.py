import os
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.colors import Normalize, LinearSegmentedColormap
import numpy as np
import sys
#from VisualizeEnvironment import Environment2DVisualizer

visualize_config = {
    "node_color": "darkgoldenrod",
    "node_size": 10,
    "edge_color": "goldenrod",
    "edge_width": 0.5,
    "colliding_color": "red",
    #"edges_as_arrows": True,
}

def visualize_coordinate_map(nodes : list, connections : list, coords : list):
    ax = plt.gca() 

    coord_map = {node : coord for node, coord in zip(nodes, coords)}

    for coord, neighbors in zip(coords, connections):
        for neighbor in neighbors:
            neighbor_coord = coord_map[neighbor]
            #if visualize_config["edges_as_arrows"]:
            #    dx = neighbor_coord[0] - coord[0] 
            #    dy = neighbor_coord[1] - coord[1] 
            #    print("dx: ", dx, " dy:", dy)
            #    ax.quiver(coord[0], coord[1], dx, dy, color=visualize_config["edge_color"], lw=visualize_config["edge_width"], scale_units='xy', scale=1)
            #else:
            ax.plot([coord[0], neighbor_coord[0]], [coord[1], neighbor_coord[1]], color=visualize_config["edge_color"], lw=visualize_config["edge_width"])

    coords_np = np.array(coords)
    ax.scatter(coords_np[:, 0], coords_np[:, 1], c=visualize_config["node_color"], s=visualize_config["node_size"]) 



