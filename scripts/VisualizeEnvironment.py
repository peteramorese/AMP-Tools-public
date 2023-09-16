import os
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, Polygon
import numpy as np
import yaml
import argparse
import sys

visualize_config = {
    "path_line_width": 2.0,
    "path_line_style": '-',
    "path_line_color": "blue",
    "show_grid": True,
    "text_font_size": 10.0,
    "show_text": False,
    "endpoint_size": 10,
    "arrow_color": "path_line_color",
    "arrow_scale": 20.0,
    "arrow_head_width": 2,
    "obstacle_color": "darkcyan",
    "collision_point_marker": "*",
    "collision_point_color": "red"
}

class Environment2DVisualizer:
    def __init__(self, x_min, x_max, y_min, y_max):
        self.__x_lims = (x_min, x_max)
        self.__y_lims = (y_min, y_max)

    def sketch_environment(self, obstacles : list, ax = None):
        if not ax:
            ax = plt.gca()
        ax.set_xlim(self.__x_lims)
        ax.set_ylim(self.__y_lims)
        
        self.__draw_obstacles(ax, obstacles)
        return ax
    
    def sketch_point(self, pt : tuple, label : str, ax = None):
        if not ax:
            ax = plt.gca() 
        ax.scatter(*pt, color="red", label=label, s=visualize_config["endpoint_size"])
        ax.text(*pt, label)

    def __draw_obstacles(self, ax, obstacles : list):
        assert isinstance(obstacles, list)
        for obstacle in obstacles:
            assert isinstance(obstacle, list)
            #edge_color = [0.5 * c for c in visualize_config["obstacle_color"]]
            patch = Polygon(obstacle, facecolor=visualize_config["obstacle_color"], edgecolor="black", lw=1)
            ax.add_patch(patch)

def new_figure():
    plt.figure()

def show_figure():
    plt.show()

def visualize_environment(bounds, obstacles, q_init = None, q_goal = None, path = None):
    visualizer = Environment2DVisualizer(*bounds)
    visualizer.sketch_environment(obstacles)
    if q_init:
        visualizer.sketch_point(q_init, "init")
    if q_goal:
        visualizer.sketch_point(q_goal, "goal")

def visualize_path(path : list, collision_points = None):
    ax = plt.gca() 
    ax.set_aspect('equal', adjustable='box')
    x_pts = list()
    y_pts = list()
    x_col_pts = list()
    y_col_pts = list()
    for waypt in path:
        x_pts.append(waypt[0])
        y_pts.append(waypt[1])
    ax.plot(x_pts, y_pts, ls=visualize_config["path_line_style"], color=visualize_config["path_line_color"], lw=visualize_config["path_line_width"], zorder=2)
    if collision_points is not None:
        for colpt in collision_points:
            x_col_pts.append(colpt[0])
            y_col_pts.append(colpt[1])
        ax.scatter(x_col_pts, y_col_pts, marker=visualize_config["collision_point_marker"], color=visualize_config["collision_point_color"], zorder=3)

