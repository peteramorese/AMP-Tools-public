import os
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, Polygon
import numpy as np
import yaml
import argparse
import sys
#from VisualizeEnvironment import Environment2DVisualizer

visualize_config = {
    "path_line_width": 2.0,
    "path_line_style": '--',
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
    "collision_point_color": "red",
    "link_width": 0.03,
    "link_color": "gray",
    "link_pad": 0.10,
    "joint_size": 10,
    "joint_color": "black",
    "eef_marker": "^",
    "eef_size": 100,
}

class LinkManipulator2DVisualizer:
    def __init__(self):
        pass
    
    def get_extended_length(self, joint_vertices : list):
        length = 0.0
        for i in range(1, len(joint_vertices)):
            src_vertex = np.array(joint_vertices[i - 1])
            dst_vertex = np.array(joint_vertices[i])
            length += np.linalg.norm(dst_vertex - src_vertex)
        return length

    def sketch_manipulator(self, joint_vertices : list, ax = None):
        if not ax:
            ax = plt.gca()
        for i in range(1, len(joint_vertices)):
            self.__sketch_link(np.array(joint_vertices[i - 1]), np.array(joint_vertices[i]), ax)
        self.__sketch_end_effector(joint_vertices[-1], ax)

    def __sketch_link(self, src_vertex : np.ndarray, dst_vertex : np.ndarray, ax):
        pad = visualize_config["link_pad"]
        width_vec = dst_vertex - src_vertex
        angle = np.arctan2(width_vec[1], width_vec[0])
        height = visualize_config["link_width"]
        width = np.linalg.norm(width_vec) - pad
        patch = FancyBboxPatch(
            (pad / 2.0, -height / 2.0), 
            width, 
            height, 
            color=visualize_config["link_color"], 
            boxstyle=f"Round, pad={pad}", 
            ec=visualize_config["joint_color"])
        patch.set_mutation_aspect(0.4)
        
        transform = mpl.transforms.Affine2D().rotate(angle) + mpl.transforms.Affine2D().translate(src_vertex[0], src_vertex[1]) + ax.transData
        patch.set_transform(transform)
        ax.add_patch(patch)
        ax.scatter(src_vertex[0], src_vertex[1], visualize_config["joint_size"], color=visualize_config["joint_color"], marker="p")

    def __sketch_end_effector(self, vertex : np.ndarray, ax):
        ax.scatter(vertex[0], vertex[1], visualize_config["eef_size"], color=visualize_config["joint_color"], marker=visualize_config["eef_marker"])
        ax.scatter(vertex[0], vertex[1], visualize_config["joint_size"], color="cyan", marker="p")
    
    def auto_bound_axes(self, joint_vertices : list, bloat_scale = 1.2, ax = None):
        if not ax:
            ax = plt.gca()
        extended_length = self.get_extended_length(joint_vertices)
        base_location = joint_vertices[0]
        ax.set_xlim((bloat_scale*(base_location[0] - extended_length), bloat_scale*(base_location[0] + extended_length)))
        ax.set_ylim((bloat_scale*(base_location[1] - extended_length), bloat_scale*(base_location[1] + extended_length)))



def visualize_manipulator(joint_vertices : list, auto_bound_axes = True):
    visualizer = LinkManipulator2DVisualizer()

    ax = plt.gca() 
    ax.grid(zorder=0, alpha=0.5)
    visualizer.sketch_manipulator(joint_vertices, ax)
    if auto_bound_axes:
        visualizer.auto_bound_axes(joint_vertices)