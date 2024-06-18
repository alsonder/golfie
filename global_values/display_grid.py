from PIL import Image, ImageDraw

# Define cell size for visualization
CELL_SIZE = 10
# Define colors for visualization
COLOR_BLOCKED = (0, 0, 0)  # Black
COLOR_UNBLOCKED = (255, 255, 255)  # White
COLOR_PATH = (0, 255, 0)  # Green
COLOR_SOURCE = (255, 0, 0)  # Red
COLOR_DESTINATION = (0, 0, 255)  # Blue

def visualize_grid(grid, src, dest, path):
    width = len(grid[0]) * CELL_SIZE
    height = len(grid) * CELL_SIZE

    img = Image.new("RGB", (width, height), COLOR_UNBLOCKED)
    draw = ImageDraw.Draw(img)

    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 0:
                cell_color = (128, 128, 128)  # Grey color
            else:
                cell_color = COLOR_UNBLOCKED
            draw.rectangle([j * CELL_SIZE, i * CELL_SIZE, (j + 1) * CELL_SIZE, (i + 1) * CELL_SIZE], fill=cell_color)

    for node in path:
        draw.rectangle([node[1] * CELL_SIZE, node[0] * CELL_SIZE, (node[1] + 1) * CELL_SIZE, (node[0] + 1) * CELL_SIZE], fill=COLOR_PATH)

    draw.rectangle([src[1] * CELL_SIZE, src[0] * CELL_SIZE, (src[1] + 1) * CELL_SIZE, (src[0] + 1) * CELL_SIZE], fill=COLOR_SOURCE)
    for point in dest:
        draw.rectangle([point[1] * CELL_SIZE, point[0] * CELL_SIZE, (point[1] + 1) * CELL_SIZE, (point[0] + 1) * CELL_SIZE], fill=COLOR_DESTINATION)

    #img.show()