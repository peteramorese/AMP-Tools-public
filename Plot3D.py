import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

if __name__ == '__main__':
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    with open('build/bin/tetrahedra.json', 'r') as file:
        data = json.load(file)
    # ax.set_facecolor('lightgrey')

    color_map = {"e": "black", "a": "teal", "b": "teal", "g": "lime", "o": "lightgrey"}
    HLP = [40, 42, 39, 37, 7, 24, 27, 14, 8, 14, 27, 24, 49, 38, 47, 5, 47, 38, 49, 24, 7, 15, 20, 26, 22, 26]

    for tetrahedron in data:
        vertices = tetrahedron['vertices']
        vertices.append(vertices[0])  # Close the loop
        color = color_map.get(chr(tetrahedron["label"]), "white")
        lw = 1
        if color == "black": lw = 0.2
        vertices = list(zip(*vertices))  # Transpose to separate x, y, z coordinates
        ax.plot(vertices[0], vertices[1], vertices[2], color=color, lw=lw)


    df = pd.read_csv('build/bin/waypoints.csv', header=None)
    x = df.iloc[:, 0].values
    y = df.iloc[:, 2].values
    z = df.iloc[:, 4].values
    ax.plot(x.flatten(), y.flatten(), z.flatten(), marker='x', linestyle='-')
    
    # half_w = 1 / 2.75
    # half_l = 0.5 / 2.75
    # for state in data:
    #     vertices = [
    #         (float(state[0]) + half_w * math.cos(float(state[2])) - half_l * math.sin(float(state[2])),
    #         float(state[1]) + half_w * math.sin(float(state[2])) + half_l * math.cos(float(state[2]))),

    #         (float(state[0]) - half_w * math.cos(float(state[2])) - half_l * math.sin(float(state[2])),
    #         float(state[1]) - half_w * math.sin(float(state[2])) + half_l * math.cos(float(state[2]))),

    #         (float(state[0]) - half_w * math.cos(float(state[2])) + half_l * math.sin(float(state[2])),
    #         float(state[1]) - half_w * math.sin(float(state[2])) - half_l * math.cos(float(state[2]))),

    #         (float(state[0]) + half_w * math.cos(float(state[2])) + half_l * math.sin(float(state[2])),
    #         float(state[1]) + half_w * math.sin(float(state[2])) - half_l * math.cos(float(state[2])))
    #     ]
    #     patch = Polygon(vertices, facecolor="seagreen", edgecolor="black", lw=0.5)
    #     ax.add_patch(patch)

    plt.show()