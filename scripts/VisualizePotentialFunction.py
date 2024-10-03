import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import numpy as np

np.seterr(invalid='ignore', divide='ignore')

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
    ax.set_title('Potential Function')

def visualize_vector_field(bounds: list, n_grid: int, u1_values: list, u2_values: list):
    assert len(bounds) == 4
    assert len(u1_values) == n_grid**2
    assert len(u2_values) == n_grid**2

    # Reshape u1 and u2 to 2D arrays for vector components
    u1 = np.array(u1_values).reshape(n_grid, n_grid)  # X component of the vector field
    u2 = np.array(u2_values).reshape(n_grid, n_grid)  # Y component of the vector field

    # Calculate the magnitudes of the vectors
    magnitudes = np.sqrt(u1**2 + u2**2)

    # Normalize vectors to have the same length (0.5 in this case)
    scale = 0.4
    u1_normalized = u1 / magnitudes * scale  # Normalize to length 0.5
    u2_normalized = u2 / magnitudes * scale

    # Create grid
    xv = np.linspace(bounds[0], bounds[1], n_grid)
    yv = np.linspace(bounds[2], bounds[3], n_grid)
    x, y = np.meshgrid(xv, yv)

    # Create figure and axis
    # fig, ax = plt.gca() 
    fig, ax = plt.subplots()
    # fig = ax.get_figure()

    # Plot vector field with normalized vectors, colored by original magnitudes
    quiver = ax.quiver(x, y, u1_normalized, u2_normalized, magnitudes, angles='xy', scale_units='xy', scale=1, cmap='viridis')

    # Add color bar to represent the magnitude
    cbar = fig.colorbar(quiver)

    # Set axis limits and labels
    ax.set_xlim(bounds[0], bounds[1])
    ax.set_ylim(bounds[2], bounds[3])
    ax.set_aspect("equal")
    ax.set_title('2D Vector Field')
