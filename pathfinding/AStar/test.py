import math

# Define points
points = [[10,10],[10,90],[20,15],[51,8],[51,49],[65,40],[80,70],[11,11],[12,12],[12,11],[11,12]]  # replace with actual values

# Distance calculation
def euclidean_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def intersects_obstacle(x, y):
    return (y >= 0) and (y < col) and (x >= 0) and (x < row) and (grid[y][x] != 0)

# Constructing the graph
n = 11  # Total points
dist = [[0]*n for _ in range(n)]
for i in range(n):
    for j in range(n):
        if not intersects_obstacle(points[i], points[j]):
            dist[i][j] = euclidean_distance(points[i], points[j])

# DP array
dp = [[float('inf')] * n for _ in range(1 << n)]
for i in range(10):
    dp[1 << i][i] = 0

# Solving the DP
for mask in range(1 << n):
    for i in range(n):
        if mask & (1 << i):
            for j in range(n):
                if not (mask & (1 << j)):
                    new_mask = mask | (1 << j)
                    dp[new_mask][j] = min(dp[new_mask][j], dp[mask][i] + dist[i][j])

# Find the shortest path ending at point 11
answer = min(dp[(1 << 10) - 1][i] + dist[i][10] for i in range(10))

row = 100
col = 100
arm_length = 20
grid = [[1 for _ in range(col)] for _ in range(row)]
for i in range(row//2 - arm_length, row//2 + arm_length + 1):
    grid[i][col//2] = 0
    grid[row//2][i] = 0


print("Minimum distance to visit all points and end at the last point is:", answer)
