import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection

# Function to simulate LiDAR data
def simulate_lidar(boat_pos, grid_size, max_range, resolution=0.1):
    grid = np.zeros(grid_size)
    for i in range(grid_size[0]):
        for j in range(grid_size[1]):
            distance = np.sqrt((i - boat_pos[0])**2 + (j - boat_pos[1])**2)
            if distance <= max_range:
                grid[i, j] = 1
    return grid

# Parameters
grid_size = (50, 50)  # Grid size
boat_pos = (grid_size[0] // 2, grid_size[1] // 2)  # Boat position at the center
max_range = 10  # Maximum LiDAR range

# Simulate LiDAR data
occupancy_grid = simulate_lidar(boat_pos, grid_size, max_range)

# Plotting
fig, ax = plt.subplots()
ax.set_aspect('equal')

# Plot the occupancy grid
ax.imshow(occupancy_grid, cmap='gray_r', origin='lower')

# Plot the boat at the center
boat = Rectangle((boat_pos[1] - 0.5, boat_pos[0] - 0.5), 1, 1, color='blue')
ax.add_patch(boat)

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Occupancy Grid with Boat and LiDAR Data')

plt.show()
