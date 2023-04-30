import numpy as np
import math

def magnitude(vector):
    return np.dot(np.array(vector),np.array(vector))

def line_ray_intersection(point_x, point_y, ray_direction, x1, y1, x2, y2):
    # Convert to numpy arrays
    ray_origin = np.array([point_x, point_y], dtype=float)
    direction_vector = np.array([math.cos(ray_direction), math.sin(ray_direction)], dtype=float)
    point1 = np.array([x1, y1], dtype=float)
    point2 = np.array([x2, y2], dtype=float)
    
    v1 = ray_origin - point1
    v2 = point2 - point1
    v3 = np.array([-direction_vector[1], direction_vector[0]])
    # do not divide by 0
    if np.dot(v2, v3) < 0.0001 and np.dot(v2, v3) > -0.0001:
        return -1
    # compute intersection
    t1 = np.cross(v2, v1) / np.dot(v2, v3)
    t2 = np.dot(v1, v3) / np.dot(v2, v3)
    if t1 >= 0.0 and t2 >= 0.0 and t2 <= 1.0:
        intersection_pt = ray_origin + t1 * direction_vector
        return math.sqrt(magnitude(intersection_pt - ray_origin))
    return -1

def point_line_distance(x1, y1, x2, y2, point_x, point_y):
    # compute vector norms
    px = x2-x1
    py = y2-y1
    norm = px**2 + py**2

    u =  ((point_x - x1) * px + (point_y - y1) * py) / float(norm)

    if u > 1:
        u = 1
    elif u < 0:
        u = 0

    # compute distance
    x = x1 + u * px
    y = y1 + u * py
    dist = math.sqrt((x - point_x)**2 + (y - point_y)**2)

    return dist