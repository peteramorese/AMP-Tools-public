import os
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, Polygon
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import Normalize, LinearSegmentedColormap
import numpy as np
import yaml
import argparse
import sys

def visualize_potential_function(bounds : list, n_grid : int, u_values : list):
    assert len(bounds) == 4
    assert len(u_values) == n_grid**2
    u_data = np.array(u_values)
    u = u_data.reshape(n_grid, n_grid)
    xv = np.linspace(bounds[0], bounds[1], n_grid)
    yv = np.linspace(bounds[2], bounds[3], n_grid)
    x, y = np.meshgrid(xv, yv)

    color_map = LinearSegmentedColormap.from_list('custom', ['darkcyan', 'indianred'], N=256)
    ax = plt.gcf().add_subplot(111, projection="3d")
    ax.plot_surface(x, y, u, cmap=color_map)