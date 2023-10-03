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


def new_figure():
    plt.figure()

def show_figure():
    plt.show()

def bloat_axis_limits(axis_limits : tuple, scale = 1.2):
    distance = abs(axis_limits[1] - axis_limits[0])
    bloat = 0.5 * distance * (scale - 1.0)
    return (axis_limits[0] - bloat, axis_limits[1] + bloat)

def visualize_polygons(polygons : list, filled, labels = None):
    if labels:
        assert len(labels) == len(polygons)
    ax = plt.gca() 
    ax.set_aspect('equal')
    x_min, x_max, y_min, y_max = None, None, None, None
    for i in range(len(polygons)):
        polygon = polygons[i]
        if labels:
            label = labels[i]
        else:
            label = None
        # Check axis bounds
        for vertex in polygon:
            if x_min is None or vertex[0] < x_min:
                x_min = vertex[0]
            if x_max is None or vertex[0] > x_max:
                x_max = vertex[0]
            if y_min is None or vertex[1] < y_min:
                y_min = vertex[1]
            if y_max is None or vertex[1] > y_max:
                y_max = vertex[1]
        # Add polygon patch
        random_color = np.random.rand(1,3)
        if filled:
            patch = Polygon(polygon, facecolor=random_color, edgecolor=visualize_config["edge_color"], lw=visualize_config["line_width"], fill=True, label=label)
        else:
            patch = Polygon(polygon, edgecolor=random_color, lw=visualize_config["line_width"], fill=False, label=label)
        ax.add_patch(patch)
    ax.set_xlim(bloat_axis_limits((x_min, x_max)))
    ax.set_ylim(bloat_axis_limits((y_min, y_max)))
    if labels:
        ax.legend()

def visualize_polygons_3d(polygons, heights_3d, filled=True):
    ax = plt.gcf().add_subplot(111, projection="3d")

    x_min, x_max, y_min, y_max = None, None, None, None
    z_min, z_max = min(heights_3d), max(heights_3d)

    # Create custom color map
    colormap = LinearSegmentedColormap.from_list('custom', ['darkcyan', 'indianred'], N=256)
    normalize = Normalize(vmin=z_min, vmax=z_max)

    for polygon, height in zip(polygons, heights_3d):
        vertices_x = np.array([vertex[0] for vertex in polygon])
        vertices_y = np.array([vertex[1] for vertex in polygon])

        # Check axis bounds
        if x_min is None or np.min(vertices_x) < x_min:
            x_min = np.min(vertices_x)
        if x_max is None or np.max(vertices_x) > x_max:
            x_max = np.max(vertices_x)
        if y_min is None or np.min(vertices_y) < y_min:
            y_min = np.min(vertices_y)
        if y_max is None or np.max(vertices_y) > y_max:
            y_max = np.max(vertices_y)

        vertices_3d = np.column_stack((vertices_x, vertices_y, [height] * len(vertices_x)))

        poly3d = [vertices_3d]

        color = colormap(normalize(height))

        ax.add_collection3d(Poly3DCollection(poly3d, facecolors=color, alpha=0.7))

    ax.set_xlim(bloat_axis_limits((x_min, x_max)))
    ax.set_ylim(bloat_axis_limits((y_min, y_max)))
    ax.set_zlim(bloat_axis_limits((z_min, z_max)))
    ax.view_init(elev=20, azim=-45)
