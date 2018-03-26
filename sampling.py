import time
import numpy as np
from shapely.geometry import Polygon, Point


def extract_polygons(data):
    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        # Extract the 4 corners of each obstacle, in counterclockwise order
        obstacle = [north - d_north, north + d_north, east - d_east, east + d_east]
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], obstacle[3]),
                   (obstacle[1], obstacle[2])]
        height = alt + d_alt
        p = Polygon(corners)
        polygons.append((p, height))

    return polygons


# Determine whether the point collides with any obstacles.
def collides(polygons, point):
    for (p, height) in polygons:
        if p.contains(Point(point)) and height >= point[2]:
            return True
    return False


def sample(data, num_samples):
    polygons = extract_polygons(data)

    # ranges for x,y,z
    x_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    x_max = np.ceil(np.max(data[:, 0] + data[:, 3]))
    y_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    y_max = np.ceil(np.max(data[:, 1] + data[:, 4]))
    z_min = 0
    z_max = np.ceil(np.max(data[:, 2] + data[:, 5]))

    x_values = np.random.uniform(x_min, x_max, num_samples)
    y_values = np.random.uniform(y_min, y_max, num_samples)
    z_values = np.random.uniform(z_min, z_max, num_samples)

    samples = np.array(list(zip(x_values, y_values, z_values)))

    print("Collision checking ...")
    feasible_points = []
    t0 = time.time()
    for point in samples:
        if not collides(polygons, point):
            feasible_points.append(point)
    print("Time taken {0} seconds ...".format(time.time() - t0))
    return feasible_points, polygons
