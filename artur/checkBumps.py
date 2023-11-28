# coding:utf-8
import math

import open3d as o3d
import numpy as np
import copy


def pcd_ground_seg_open3d(scan):
    """ Open3D also supports segmentation of geometric primitives from point clouds using RANSAC.
    """
    pcd = copy.deepcopy(scan)
    ground_model, ground_indexes = scan.segment_plane(distance_threshold=0.03,
                                                      ransac_n=3,
                                                      num_iterations=100)
    ground_indexes = np.array(ground_indexes)
    ground = pcd.select_by_index(ground_indexes)
    rest = pcd.select_by_index(ground_indexes, invert=True)
    # ground.paint_uniform_color(config['ground_color'])
    # rest.paint_uniform_color(config['rest_color'])
    return ground, rest


def distance_plane_to_point(x1, y1, z1, a, b, c, d) -> float:

    d = abs((a * x1 + b * y1 + c * z1 + d))
    e = (math.sqrt(a * a + b * b + c * c))
    try:
        return d / e
    except FloatingPointError:
        return 0


def create_plane(point1, point2, point3) -> tuple:
    p1 = np.array(point1)
    p2 = np.array(point2)
    p3 = np.array(point3)

    # These two vectors are in the plane
    v1 = p3 - p1
    v2 = p2 - p1

    # the cross product is a vector normal to the plane
    cp = np.cross(v1, v2)
    a, b, c = cp

    # This evaluates a * x3 + b * y3 + c * z3 which equals d
    d = np.dot(cp, p3)

    return a, b, c, d


def equation_plane(point1, point2, point3):
    x1, y1, z1 = point1
    x2, y2, z2 = point2
    x3, y3, z3 = point3
    a1 = x2 - x1
    b1 = y2 - y1
    c1 = z2 - z1
    a2 = x3 - x1
    b2 = y3 - y1
    c2 = z3 - z1
    a = b1 * c2 - b2 * c1
    b = a2 * c1 - a1 * c2
    c = a1 * b2 - b1 * a2
    d = (- a * x1 - b * y1 - c * z1)
    return a, b, c, d


def get_three_points(arr) -> tuple:
    highest_x, lowest_x, highest_y = 0, 0, 0
    for i in range(1, len(arr)):

        if arr[i][0] > arr[highest_x][0]:
            highest_x = i

        if arr[i][0] < arr[lowest_x][0]:
            lowest_x = i

        if arr[i][1] > arr[highest_y][1]:
            highest_y = i

    return arr[highest_x], arr[lowest_x], arr[highest_y]


if __name__ == "__main__":
    np.seterr('raise')


    point = o3d.io.read_point_cloud("../data/laserscanner2/04083.pcd")
    o3d.visualization.draw_geometries([point])
    ground, rest = pcd_ground_seg_open3d(point)
    # o3d.visualization.draw_geometries([ground])

    ground_points = np.asarray(ground.points)

    plane_points = get_three_points(arr=ground_points)

    a, b, c, d = create_plane(plane_points[0], plane_points[1], plane_points[2])
    a1, b1, c1, d1 = equation_plane(plane_points[0], plane_points[1], plane_points[2])

    all_points = np.asarray(point.points)
    for point in all_points:
        distance = distance_plane_to_point(point[0], point[1], point[2], a1, b1, c1, d1)

        if distance > 3 and ground_points[0][2] < point[2]:
            print(distance)

    # print(ground_points)
