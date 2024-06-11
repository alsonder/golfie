

#quick midlertidig løsning med mr.ai
def fill_polygon(polygon_points):
    from collections import defaultdict

    def bresenham_line(x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return points

    # Generate all edge points
    edge_points = []
    num_points = len(polygon_points)
    for i in range(num_points):
        x0, y0 = polygon_points[i]
        x1, y1 = polygon_points[(i + 1) % num_points]
        edge_points.extend(bresenham_line(x0, y0, x1, y1))

    # Sort edge points by y-coordinate
    edge_points = sorted(edge_points, key=lambda p: (p[1], p[0]))
    y_min = edge_points[0][1]
    y_max = edge_points[-1][1]

    # Create a dictionary to hold the x-coordinates for each y-coordinate
    scanline = defaultdict(list)
    for x, y in edge_points:
        scanline[y].append(x)

    # Fill the polygon
    filled_points = []
    for y in range(y_min, y_max + 1):
        if y in scanline:
            x_points = sorted(scanline[y])
            for x in range(x_points[0], x_points[-1] + 1):
                filled_points.append((x, y))

    return filled_points

# Usage example
polygon_points = [(323, 240), (313, 236), (302, 253), (277, 245), (275, 252),
                  (295, 264), (284, 288), (292, 291), (305, 272), (329, 281),
                  (331, 274), (309, 259)]

filled_polygon_points = fill_polygon(polygon_points)
