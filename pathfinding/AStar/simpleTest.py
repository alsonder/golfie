from Astar_and_ShortestSpanning import nearest_neighbor, nearest_neighbor_simplified, gridCreation, add_point_to_grid
from displayer import visualize_grid

           # 1        2        3        4       5       6        7       8     9        10         11
#points = [(10, 10), (90,90),(20,30),(30,20),(40,40),(90,70),(70,90),(90,10), (10,90),  (49,51), (98,98)]
points = [(10,10),(98,20),(96,50), (98,50)]

ROW, COL = 100,100

#points = [(40,10),(30,20),(20,10),  (511,383), (10, 10),(600,600)]

#format of grid is grid[column][row]

down, right = True, False
grid = gridCreation(ROW, COL, 20, (down,right))
print("grid created")
#print("ROW: ",len(grid))
#print("COL: ",len(grid[0]))

add_point_to_grid(grid, ([(11, 10), (10, 11), (11, 11), (11,9), (10,9)]))

nn_path, nn_order = nearest_neighbor(grid,points)
#simple_order = nearest_neighbor_simplified(points)
print("path has been found")
print("starting points:", points)
print("advanced order: ", nn_order)
#print("simple   order: ", simple_order)
print(nn_path)
visualize_grid(grid, nn_path[0], points, nn_path)
