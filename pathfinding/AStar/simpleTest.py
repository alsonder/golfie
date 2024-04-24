from Astar_and_ShortestSpanning import nearest_neighbor, gridCreation, gridCreation
from displayer import visualize_grid

           # 1        2        3        4       5       6        7       8     9        10         11
#points = [(10, 10), (90,90),(20,30),(30,20),(40,40),(90,70),(70,90),(90,10), (10,90),  (49,51), (99,99)]

ROW, COL = 1024,768

points = [(10,10), (190,90), (170,40), (1000,760)]

#format of grid is grid[column][row]

down, right = True, False
grid = gridCreation(ROW, COL, 20, (down,right))
print("grid created")
#print("ROW: ",len(grid))
#print("COL: ",len(grid[0]))

nn_path, nn_order = nearest_neighbor(grid,points)
print("path has been found")
visualize_grid(grid, nn_path[0], points, nn_path)
