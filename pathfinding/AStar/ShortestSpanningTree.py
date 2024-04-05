from itertools import permutations
import math

def intersects_obstacle(point1, point2):

    # Define the cross obstacle coordinates
    cross_obstacle = {(50,50)}
    for i in range(20):
        if i != 10:
            new_points = [(40+i,50), (50,40+i)]
            cross_obstacle.update(new_points)

    # Check if either point is part of the obstacle
    if tuple(point1) in cross_obstacle or tuple(point2) in cross_obstacle:
        return True
    
    # Simple strategy: Check if the path's bounding box intersects with the obstacle's region
    x_min, x_max = min(point1[0], point2[0]), max(point1[0], point2[0])
    y_min, y_max = min(point1[1], point2[1]), max(point1[1], point2[1])

    for x in range(x_min, x_max + 1):
        for y in range(y_min, y_max + 1):
            if (x, y) in cross_obstacle:
                return True
    return False

def distance(point1, point2):
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def total_distance(points):
    """Calculate the total distance of a path, considering the obstacle."""
    total_dist = 0
    for i in range(len(points) - 1):
        if intersects_obstacle(points[i], points[i+1]):
            return float('inf')  # Infinite distance if intersects with obstacle
        total_dist += distance(points[i], points[i+1])
    return total_dist

# Example points
points = [[10, 30],  [80, 10],[90, 90], [0, 0],[20, 2], [51,51], [49,49]]

# Find the shortest path
all_permutations = permutations(points)
shortest_path = None
min_distance = float('inf')

for order in all_permutations:
    dist = total_distance(order)
    if dist < min_distance:
        min_distance = dist
        shortest_path = order

print(f"Shortest path: {shortest_path} with distance: {min_distance}")
