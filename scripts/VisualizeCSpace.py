import os
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, Polygon
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.colors import Normalize, LinearSegmentedColormap
import numpy as np
import yaml
import argparse
import sys

visualize_config = {
    "line_width": 2.0,
    "line_style": '--',
    "line_color": "blue",
    "edge_color": "black",
    "text_font_size": 10.0,
}

def visualize_grid_cspace_2d(x0_cells : int, x1_cells : int, bounds : list, dense_bits : list):
    assert len(bounds) == 4
    assert len(dense_bits) == x0_cells * x1_cells
    data = np.zeros((x0_cells, x1_cells), dtype=bool)

    # Convert the dense data
    for i in range(x0_cells):
        for j in range(x1_cells):
            wrapped_idx = j * x0_cells + i
            data[i][j] = dense_bits[wrapped_idx]
    
    color_map = LinearSegmentedColormap.from_list('custom', ['darkcyan', 'indianred'], N=256)
    ax = plt.gca()
    ax.imshow(data.transpose(), cmap=color_map, interpolation='none', origin='upper')
    ax.invert_yaxis()
