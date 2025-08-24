import os
import math
from matplotlib.patches import Polygon

def get_vertices(state : tuple, dimensions : tuple, isCar : bool):
    half_l = dimensions[1] / 2
    half_w = dimensions[0] / 2
    theta = state[4]
    ref_x = state[0] + half_w * math.cos(theta) if isCar else state[0]
    ref_y = state[1] + half_w * math.sin(theta) if isCar else state[1]
    vertices = [
        (ref_x + half_w * math.cos(theta) - half_l * math.sin(theta),
        ref_y + half_w * math.sin(theta) + half_l * math.cos(theta)),

        (ref_x - half_w * math.cos(theta) - half_l * math.sin(theta),
        ref_y - half_w * math.sin(theta) + half_l * math.cos(theta)),

        (ref_x - half_w * math.cos(theta) + half_l * math.sin(theta),
        ref_y - half_w * math.sin(theta) - half_l * math.cos(theta)),

        (ref_x + half_w * math.cos(theta) + half_l * math.sin(theta),
        ref_y + half_w * math.sin(theta) - half_l * math.cos(theta))
    ]
    patch = Polygon(vertices, facecolor='mediumaquamarine', edgecolor="black", lw=0.5)
    return patch

def get_most_recent(solutions_dir):
    subdirs = [d for d in os.listdir(solutions_dir) if os.path.isdir(os.path.join(solutions_dir, d))]

    # Sort the subdirectories by name (assuming names are in YYYY_MM_DD_HH_MM_SS format)
    subdirs.sort()

    # Get the most recent directory
    return os.path.join(solutions_dir, subdirs[-1]) if subdirs else None