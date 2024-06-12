from .create_path import calculate_distance_to_wall

def gridCreation(row, col, obstacles):
        # Define the new grid with all elements initialized to 1 (open field)
    grid = [[1 for _ in range(col)] for _ in range(row)]

    # Add obstacles around the edges of the grid
    for i in range(row):
        grid[i][0] = 0
        grid[i][col-1] = 0

    for j in range(col):
        grid[0][j] = 0
        grid[row-1][j] = 0

    for point in obstacles:
        x, y = point
        grid[y][x] = 0

    print("Grid Creation           | Successful")
    weighted = calculate_distance_to_wall(grid)
    return grid, weighted
