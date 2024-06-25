import heapq
import math
import numpy as np

# Define the Cell class
class Cell:
    def __init__(self):
        self.parent_i = 0  # Parent cell's row index
        self.parent_j = 0  # Parent cell's column index
        self.f = float('inf')  # Total cost of the cell (g + h)
        self.g = float('inf')  # Cost from start to this cell
        self.h = 0  # Heuristic cost from this cell to destination

def is_valid(row, col, ROW, COL):
    return (row >= 0) and (row < ROW) and (col >= 0) and (col < COL)

# Check if a cell is unblocked
def is_unblocked(grid, row, col):
    return grid[row][col] == 1

# Check if a cell is the destination
def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]

# Calculate the heuristic value of a cell (Euclidean distance to destination)
def calculate_h_value(row, col, dest, distance_to_wall):
    return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5 + 80 * math.exp(-distance_to_wall / 75)

from collections import deque

def calculate_distance_to_wall(grid):
    ROW = len(grid)
    COL = len(grid[0])
    distance_to_wall = [[float('inf') for _ in range(COL)] for _ in range(ROW)]
    queue = deque()

    # Initialize the queue with all wall cells and set their distance to 0
    for i in range(ROW):
        for j in range(COL):
            if grid[i][j] == 0:
                distance_to_wall[i][j] = 0
                queue.append((i, j))

    # Perform BFS from all wall cells
    while queue:
        i, j = queue.popleft()
        current_distance = distance_to_wall[i][j]

        # Explore all 8 possible directions
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dir in directions:
            new_i = i + dir[0]
            new_j = j + dir[1]

            if is_valid(new_i, new_j, ROW, COL) and distance_to_wall[new_i][new_j] > current_distance + 1:
                distance_to_wall[new_i][new_j] = current_distance + 1
                queue.append((new_i, new_j))
    print("Weights Added to Grid   | Successful")
    return distance_to_wall

# Implement the A* search algorithm
def a_star_search(grid, src, dest, distance_to_wall):
    ROW = len(grid)
    COL = len(grid[0])

    # Check if the source and destination are valid
    if not is_valid(src[0], src[1], ROW, COL) or not is_valid(dest[0], dest[1], ROW, COL):
        print("Source or destination is invalid: ", src, dest)
        return None

    # Check if the source and destination are unblocked
    if not is_unblocked(grid, src[0], src[1]) or not is_unblocked(grid, dest[0], dest[1]):
        print("Source or the destination is blocked")
        return None

    # Check if we are already at the destination
    if is_destination(src[0], src[1], dest):
        print("We are already at the destination")
        return []

    # Initialize the closed list (visited cells)
    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
    # Initialize the details of each cell
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

    # Initialize the start cell details
    i = src[0]
    j = src[1]
    cell_details[i][j].f = 0
    cell_details[i][j].g = 0
    cell_details[i][j].h = 0
    cell_details[i][j].parent_i = i
    cell_details[i][j].parent_j = j

    # Initialize the open list (cells to be visited) with the start cell
    open_list = []
    heapq.heappush(open_list, (0.0, i, j))

    # Initialize the flag for whether destination is found
    found_dest = False

    # Main loop of A* search algorithm
    while len(open_list) > 0:
        # Pop the cell with the smallest f value from the open list
        p = heapq.heappop(open_list)

        # Mark the cell as visited
        i = p[1]
        j = p[2]
        closed_list[i][j] = True

        # For each direction, check the successors
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dir in directions:
            new_i = i + dir[0]
            new_j = j + dir[1]

            # If the successor is valid, unblocked, and not visited
            if is_valid(new_i, new_j, ROW, COL) and is_unblocked(grid, new_i, new_j) and not closed_list[new_i][new_j]:
                # If the successor is the destination
                if is_destination(new_i, new_j, dest):
                    # Set the parent of the destination cell
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    found_dest = True
                    break
                else:
                    # Calculate the new f, g, and h values
                    g_new = cell_details[i][j].g + 1.0
                    h_new = calculate_h_value(new_i, new_j, dest, distance_to_wall[new_i][new_j])
                    f_new = g_new + h_new

                    # If the cell is not in the open list or the new f value is smaller
                    if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                        # Add the cell to the open list
                        heapq.heappush(open_list, (f_new, new_i, new_j))
                        # Update the cell details
                        cell_details[new_i][new_j].f = f_new
                        cell_details[new_i][new_j].g = g_new
                        cell_details[new_i][new_j].h = h_new
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j

        if found_dest:
            break

    # If the destination is not found after visiting all cells
    if not found_dest:
        print("Failed to find the destination cell")
        return None

    # Trace and return the path from source to destination
    path = []
    row = dest[0]
    col = dest[1]
    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        row = temp_row
        col = temp_col
    path.append((row, col))
    path.reverse()  # Reverse the path to start from the source

    print("Path To Node Found      | Successful")
    return path

def nearest_neighbor(grid, points, distance_to_wall=0):

    def calculate_h_value(y1, x1, point):
        y2, x2 = point
        return abs(y1 - y2) + abs(x1 - x2)

    start = points[0]  # Start from the first point
    end = points[-1]   # Ensure ending at the last point
    other_points = points[1:-1]  # Points in between

    path = []
    unvisited = set(other_points)
    current_point = start
    visit_order = [current_point]
    if(distance_to_wall==0):
        distance_to_wall = calculate_distance_to_wall(grid)

    # Travel to the nearest unvisited point each time
    while unvisited:
        next_point = min(unvisited, key=lambda x: calculate_h_value(current_point[1], current_point[0], x))
        segment_path = a_star_search(grid, current_point, next_point, distance_to_wall)
        if segment_path is None:
            print("Failed to find path to", next_point)
            return None
        path.extend(segment_path[:-1])  # path without repeating last node
        current_point = next_point
        visit_order.append(current_point)
        unvisited.remove(current_point)
    
    # Finally, move from the last visited point to the end point
    final_segment = a_star_search(grid, current_point, end, distance_to_wall)
    if final_segment is None:
        print("Failed to find path to", end)
        return None
    path.extend(final_segment)
    visit_order.append(end)
    
    return path, visit_order

def nearest_neighbor_simplified(points):
    def calculate_distance(point1, point2):
        point1 = np.array(point1, dtype=np.int64)
        point2 = np.array(point2, dtype=np.int64)
        return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

    start = points[0]  # Start from the first point
    #end = points[-1]   # Ensure ending at the last point
    other_points = points[1:]  # Points in between

    visit_order = [start]
    unvisited = set(other_points)
    current_point = start

    # Travel to the nearest unvisited point each time
    while unvisited:
        next_point = min(unvisited, key=lambda x: calculate_distance(current_point, x))
        visit_order.append(next_point)
        current_point = next_point
        unvisited.remove(next_point)

    # Finally, add the end point
    #visit_order.append(end)

    return visit_order[1:]


def calculate_turn_points(path, threshold=1):
    # Define the possible directions
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    
    def get_direction(p1, p2):
        return (p2[0] - p1[0], p2[1] - p1[1])

    def distance(p1, p2):
        return ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5

    # Initialize the list of turn points
    turn_points = []

    if len(path) < 2:
        return path  # Not enough points to have any turns

    # Start by adding the first point as it is always a turn point
    turn_points.append(path[0])

    # Iterate over the path to detect turns
    for i in range(1, len(path) - 1):
        # Get the direction of the current segment and the next segment
        direction1 = get_direction(path[i - 1], path[i])
        direction2 = get_direction(path[i], path[i + 1])

        # Check if the direction changes
        if direction1 != direction2:
            turn_points.append(path[i])

    # Add the last point as it is always a turn point
    turn_points.append(path[-1])

    # Remove close turn points
    filtered_turn_points = []
    last_turn_point = turn_points[0]
    filtered_turn_points.append(last_turn_point)

    for point in turn_points[1:]:
        if distance(last_turn_point, point) > threshold:
            filtered_turn_points.append(point)
        last_turn_point = point

    return filtered_turn_points


def are_points_close(point1, point2, threshold=15):
    """
    Check if two points are within a certain distance (threshold) from each other.

    Args:
    point1 (tuple): The first point as (x, y).
    point2 (tuple): The second point as (x, y).
    threshold (int): The distance threshold.

    Returns:
    bool: True if the points are within the threshold distance, False otherwise.
    """
    # Calculate the Euclidean distance between the two points
    distance = math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
    
    # Check if the distance is within the threshold
    return distance <= threshold

    
