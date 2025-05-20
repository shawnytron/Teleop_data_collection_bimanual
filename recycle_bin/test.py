import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Define the matrices
vr_to_global_mat = np.asarray([
    [-0.628429, -0.694163, 0.351017, -0.249769],
    [-0.684221, 0.278634, -0.673947, 0.0486907],
    [0.370024, -0.663702, -0.650063, -0.135777],
    [0.0, 0.0, 0.0, 1.0]
])

global_to_env_mat = np.asarray([
    [ 0., -1.,  0.,  0.],
    [-1.,  0.,  0.,  0.],
    [ 0.,  0., -1.,  0.],
    [ 0.,  0.,  0.,  1.]
])

rot_mat = np.asarray([
    [0.5, -0.5, 0.5, 0.0],
    [0.5, 0.5, -0.5, 0.0],
    [0.5, 0.5, 0.5, 0.0],
    [0.0, 0.0, 0.0, 1.0]
])

# Function to interpolate between two matrices
def interpolate_matrices(start_matrix, end_matrix, alpha):
    return start_matrix * (1 - alpha) + end_matrix * alpha

# Set up the plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Function to update the plot for each frame
def update(frame):
    # For the first 50 frames, interpolate from vr_to_global_mat to global_to_env_mat
    if frame < 50:
        alpha = frame / 50
        current_matrix = interpolate_matrices(vr_to_global_mat, global_to_env_mat, alpha)
    # For the next 50 frames, interpolate from global_to_env_mat to rot_mat
    else:
        alpha = (frame - 50) / 50
        current_matrix = interpolate_matrices(global_to_env_mat, rot_mat, alpha)

    # Clear previous plot data
    ax.clear()
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)
    
    # Create a set of vectors to show the transformation (representing the matrix's columns)
    for i in range(3):  # Show first three columns (3D vectors)
        vector = current_matrix[:, i]
        ax.quiver(0, 0, 0, vector[0], vector[1], vector[2], color='r' if i == 0 else ('g' if i == 1 else 'b'))
    
    # Set axis labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    return ax.quiver(0, 0, 0, vector[0], vector[1], vector[2])

# Create the animation
ani = FuncAnimation(fig, update, frames=np.arange(0, 101, 1), interval=100)

# Save the animation as a video file
ani.save('matrix_interpolation_animation.mp4', writer='ffmpeg', fps=30)

plt.show()
