import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance

class UAV:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Obstacle:
    def __init__(self, x, y, z, radius):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius

def calculate_distance(uav, obstacle):
    return distance.euclidean([uav.x, uav.y, uav.z], [obstacle.x, obstacle.y, obstacle.z])

def check_collision(uav, obstacle):
    return calculate_distance(uav, obstacle) <= obstacle.radius

def path_planning(uav, obstacles, start, end):
    # Create a grid for the 3D space
    grid_size = 10
    grid = np.zeros((grid_size, grid_size, grid_size))

    # Set the start and end points
    start_point = UAV(start[0], start[1], start[2])
    end_point = UAV(end[0], end[1], end[2])

    # Mark the obstacles in the grid
    for obstacle in obstacles:
        for i in range(grid_size):
            for j in range(grid_size):
                for k in range(grid_size):
                    if distance.euclidean([i, j, k], [obstacle.x, obstacle.y, obstacle.z]) <= obstacle.radius:
                        grid[i, j, k] = 1

    # Perform the path planning
    current_point = start_point
    path = [current_point]
    while current_point != end_point:
        # Find the next point in the grid
        for i in range(grid_size):
            for j in range(grid_size):
                for k in range(grid_size):
                    if grid[i, j, k] == 0 and distance.euclidean([i, j, k], current_point) < distance.euclidean([i, j, k], next_point):
                        next_point = UAV(i, j, k)
                        grid[i, j, k] = 1
                        break

        # Check for collisions
        if check_collision(next_point, obstacles):
            print("Collision detected!")
            return None

        # Update the current point and add it to the path
        current_point = next_point
        path.append(current_point)

    return path

# Example usage
uav = UAV(0, 0, 0)
obstacles = [Obstacle(1, 1, 1, 0.5), Obstacle(2, 2, 2, 0.5)]
start = (0, 0, 0)
end = (10, 10, 10)

path = path_planning(uav, obstacles, start, end)
if path is not None:
    plt.plot([p.x for p in path], [p.y for p in path], [p.z for p in path])
    plt.show()