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
    "path_line_style": '--',
    "path_line_color": "seagreen",
    "path_point_marker": "o",
    "show_grid": True,
    "text_font_size": 10.0,
    "show_text": False,
    "endpoint_size": 40,
    "arrow_color": "path_line_color",
    "arrow_scale": 20.0,
    "arrow_head_width": 2,
    "obstacle_color": "indianred",
    "collision_point_marker": "*",
    "collision_point_color": "red",
    "init_color": "indianred",
    "goal_color": "seagreen",
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
        ax.set_aspect("equal")
        
        self.__draw_obstacles(ax, obstacles)
        return ax
    
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

def visualize_environment(bounds, obstacles, path = None):
    visualizer = Environment2DVisualizer(*bounds)
    visualizer.sketch_environment(obstacles)

def show_endpoints(q_init : tuple, q_goal : tuple, random_color = False, tag = None):
    ax = plt.gca() 
    init_label = "init" if tag is None else "init" + tag
    goal_label = "goal" if tag is None else "goal" + tag
    if random_color:
        color = np.random.rand(3)
        init_color = color
        goal_color = color
    else:
        init_color = visualize_config["init_color"]
        goal_color = visualize_config["goal_color"]
    ax.scatter(*q_init, color=init_color, label=init_label, s=visualize_config["endpoint_size"])
    ax.text(*q_init, init_label)
    ax.scatter(*q_goal, color=goal_color, label=goal_label, s=visualize_config["endpoint_size"])
    ax.text(*q_goal, goal_label)


def visualize_path(path : list, collision_points = None):
    ax = plt.gca() 
    x_pts = list()
    y_pts = list()
    x_col_pts = list()
    y_col_pts = list()
    for waypt in path:
        x_pts.append(waypt[0])
        y_pts.append(waypt[1])
    # ax.scatter(x_pts, y_pts, marker=visualize_config["path_point_marker"], color=visualize_config["path_line_color"], zorder=2)
    ax.plot(x_pts, y_pts, ls=visualize_config["path_line_style"], marker=visualize_config["path_point_marker"], color=visualize_config["path_line_color"], zorder=2)
    if collision_points is not None:
        for colpt in collision_points:
            x_col_pts.append(colpt[0])
            y_col_pts.append(colpt[1])
        ax.scatter(x_col_pts, y_col_pts, marker=visualize_config["collision_point_marker"], color=visualize_config["collision_point_color"], zorder=3)

