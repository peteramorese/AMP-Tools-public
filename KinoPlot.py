import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math
import csv
import json

if __name__ == '__main__':
    # workspace1 = [
    #     "POLYGON((1.000000 1.000000,2.000000 1.000000,2.000000 5.000000,1.000000 5.000000,1.000000 1.000000))",
    #     "POLYGON((3.000000 3.000000,4.000000 3.000000,4.000000 12.000000,3.000000 12.000000,3.000000 3.000000))",
    #     "POLYGON((3.000000 12.000000,12.000000 12.000000,12.000000 13.000000,3.000000 13.000000,3.000000 12.000000))",
    #     "POLYGON((12.000000 5.000000,13.000000 5.000000,13.000000 13.000000,12.000000 13.000000,12.000000 5.000000))",
    #     "POLYGON((6.000000 5.000000,12.000000 5.000000,12.000000 6.000000,6.000000 6.000000,6.000000 5.000000))"
    # ]
    # workspace2 = [
    #     "POLYGON((-6.000000 -6.000000,25.000000 -6.000000,25.000000 -5.000000,-6.000000 -5.000000,-6.000000 -6.000000))",
    #     "POLYGON((-6.000000 5.000000,30.000000 5.000000,30.000000 6.000000,-6.000000 6.000000,-6.000000 5.000000))",
    #     "POLYGON((-6.000000 -5.000000,-5.000000 -5.000000,-5.000000 5.000000,-6.000000 5.000000,-6.000000 -5.000000))",
    #     "POLYGON((4.000000 -5.000000,5.000000 -5.000000,5.000000 1.000000,4.000000 1.000000,4.000000 -5.000000))",
    #     "POLYGON((9.000000 0.000000,10.000000 0.000000,10.000000 5.000000,9.000000 5.000000,9.000000 0.000000))",
    #     "POLYGON((14.000000 -5.000000,15.000000 -5.000000,15.000000 1.000000,14.000000 1.000000,14.000000 -5.000000))",
    #     "POLYGON((19.000000 0.000000,20.000000 0.000000,20.000000 5.000000,19.000000 5.000000,19.000000 0.000000))",
    #     "POLYGON((24.000000 -5.000000,25.000000 -5.000000,25.000000 1.000000,24.000000 1.000000,24.000000 -5.000000))",
    #     "POLYGON((29.000000 0.000000,30.000000 0.000000,30.000000 5.000000,29.000000 5.000000,29.000000 0.000000))",
    # ]
    # workspace3 = [
    #     "POLYGON((10.000000 2.000000,10.000000 0.000000,12.000000 0.000000))",
    #     "POLYGON((12.000000 2.000000,10.000000 2.000000,12.000000 0.000000))",
    #     "POLYGON((1.000000 1.000000,2.000000 1.000000,1.000000 5.000000))",
    #     "POLYGON((12.000000 5.000000,12.000000 6.000000,6.000000 6.000000))",
    #     "POLYGON((11.000000 11.000000,9.000000 11.000000,11.000000 9.000000))",
    #     "POLYGON((11.000000 9.000000,9.000000 11.000000,9.000000 9.000000))",
    #     "POLYGON((4.000000 12.000000,12.000000 12.000000,12.000000 13.000000))",
    #     "POLYGON((2.000000 1.000000,2.000000 5.000000,1.000000 5.000000))",
    #     "POLYGON((12.000000 13.000000,12.000000 12.000000,13.000000 13.000000))",
    #     "POLYGON((4.000000 12.000000,3.000000 13.000000,3.000000 12.000000))",
    #     "POLYGON((3.000000 12.000000,4.000000 3.000000,4.000000 12.000000))",
    #     "POLYGON((3.000000 12.000000,3.000000 3.000000,4.000000 3.000000))",
    #     "POLYGON((12.000000 13.000000,3.000000 13.000000,4.000000 12.000000))",
    #     "POLYGON((13.000000 13.000000,12.000000 12.000000,13.000000 5.000000))",
    #     "POLYGON((12.000000 12.000000,12.000000 6.000000,13.000000 5.000000))",
    #     "POLYGON((12.000000 5.000000,13.000000 5.000000,12.000000 6.000000))",
    #     "POLYGON((6.000000 5.000000,12.000000 5.000000,6.000000 6.000000))",
    # ]

    plt.figure()
    ax = plt.gca() 
    ax.set_facecolor('lightgrey')
   
    with open('build/bin/triangles.json') as f: triangles = json.load(f)
    color_map = {"e": "white", "a": "teal", "b": "teal", "g": "lime", "o": "lightgrey"}
    HLP = [40, 42, 39, 37, 7, 24, 49, 38, 47, 5, 47, 38, 49, 24, 27, 14, 8, 14, 27, 24, 7, 15, 20, 26, 22, 26]

    for triangle in triangles:
        vertices = triangle["vertices"]
        color = color_map.get(chr(triangle["observation"]), "white")
        patch = Polygon(vertices, facecolor=color, edgecolor="black", lw=1)
        ax.add_patch(patch)
        if triangle["index"] in HLP[:-1]:
            ax.add_patch(Polygon(vertices, facecolor="lightcoral", alpha=0.75))


    # Convert polygon strings to lists of obstacles
    # obstacles = []
    # for poly_str in workspace1:
    #     coords = poly_str.split('((')[1].split('))')[0].split(',')
    #     polygon_points = [tuple(map(float, coord.split())) for coord in coords]
    #     obstacles.append(polygon_points)

    with open('build/bin/waypoints.csv', newline='') as csvfile:
        reader = csv.reader(csvfile)
        data = list(reader)
    ax.set_aspect('equal', adjustable='box')
    x_pts = list()
    y_pts = list()
    # for waypt in data:
    #     x_pts.append(float(waypt[0]))
    #     y_pts.append(float(waypt[1]))
    # print(obstacles)
    #ax.scatter(x_pts, y_pts, ls=visualize_config["path_line_style"], color=visualize_config["path_line_color"], lw=visualize_config["path_line_width"], zorder=2)
    #ax.scatter(x_pts, y_pts, marker=visualize_config["path_point_marker"], color=visualize_config["path_line_color"], zorder=2)
    ax.plot(0, 0)
    # ax.set_xlim((-1, 15))
    # ax.set_ylim((-1, 15))
    
    half_w = 1 / 2.75
    half_l = 0.5 / 2.75
    for state in data:
        vertices = [
            (float(state[0]) + half_w * math.cos(float(state[2])) - half_l * math.sin(float(state[2])),
            float(state[1]) + half_w * math.sin(float(state[2])) + half_l * math.cos(float(state[2]))),

            (float(state[0]) - half_w * math.cos(float(state[2])) - half_l * math.sin(float(state[2])),
            float(state[1]) - half_w * math.sin(float(state[2])) + half_l * math.cos(float(state[2]))),

            (float(state[0]) - half_w * math.cos(float(state[2])) + half_l * math.sin(float(state[2])),
            float(state[1]) - half_w * math.sin(float(state[2])) - half_l * math.cos(float(state[2]))),

            (float(state[0]) + half_w * math.cos(float(state[2])) + half_l * math.sin(float(state[2])),
            float(state[1]) + half_w * math.sin(float(state[2])) - half_l * math.cos(float(state[2])))
        ]
        patch = Polygon(vertices, facecolor="seagreen", edgecolor="black", lw=0.5)
        ax.add_patch(patch)

    ax.set_aspect("equal")
    plt.show()