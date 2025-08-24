import os
import matplotlib.pyplot as plt

def new_figure():
    plt.figure()

def show_figure():
    plt.show()

def save_figures(directory, format):
    base_path = os.path.join("../../figures", directory)
    os.makedirs(base_path, exist_ok=True)

    figures = [plt.figure(num) for num in plt.get_fignums()]
    for i, fig in enumerate(figures, start=1):
        filename = os.path.join(base_path, f"figure_{i}.{format}")
        fig.savefig(filename)