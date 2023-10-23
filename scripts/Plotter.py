import os
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.colors import Normalize, LinearSegmentedColormap
import numpy as np
import sys
#from VisualizeEnvironment import Environment2DVisualizer

visualize_config = {
    "bar_color": "darkcyan",
}

def make_bar_graph(values : list, labels : list, title : str, xlabel : str, ylabel : str):
    ax = plt.gca() 
    ax.bar(labels, values, color=visualize_config["bar_color"])
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)

def make_box_plot(data : list, labels : list, title : str, xlabel : str, ylabel : str):
    ax = plt.gca() 
    ax.boxplot(data, labels=labels)
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)

