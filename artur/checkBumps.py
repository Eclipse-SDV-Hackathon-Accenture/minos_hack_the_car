import math

import open3d as o3d
import numpy as np
import copy


def pcd_ground_seg_open3d(scan):
    """ Open3D also supports segmentation of geometric primitives from point clouds using RANSAC.
    """
    pcd = copy.deepcopy(scan)
    ground_model, ground_indexes = scan.segment_plane(distance_threshold=0.3,
                                                      ransac_n=3,
                                                      num_iterations=10000)
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


def rotate(model, degrees):
    angle = degrees * np.pi / 180

    # Rotation matrix
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0],
                                [np.sin(angle), np.cos(angle), 0],
                                [0, 0, 1]])

    # Apply the rotation
    rotated_points = np.dot(model.points, rotation_matrix.T)

    # Update the point cloud with the rotated points
    model.points = o3d.utility.Vector3dVector(rotated_points)


if __name__ == "__main__":
    np.seterr('raise')

    pcd = o3d.io.read_point_cloud("../data/car/test.pcd")
    rotate(pcd, 55)

    all_points = np.asarray(pcd.points)
    all_colors = np.asarray(pcd.colors)

    # Filter the points according to the size
    filtered_points = []
    width = 20
    length = 50

    for i in range(0, len(all_points)):
        point = all_points[i]
        if -(width / 2) <= point[0] <= width / 2 and -length <= point[1] <= 0 and point[2] <= 0:
            filtered_points.append(point)

    filtered_model = o3d.geometry.PointCloud()
    filtered_model.points = o3d.utility.Vector3dVector(np.array(filtered_points))

    # retrieving ground
    ground, rest = pcd_ground_seg_open3d(filtered_model)
    ground.paint_uniform_color((0.7, 0.7, 0.7))
    ground_points = np.asarray(ground.points)

    o3d.visualization.draw_geometries([ground])

    # create mathematical plain
    plane_points = get_three_points(arr=ground_points)

    a, b, c, d = equation_plane(plane_points[0], plane_points[1], plane_points[2])

    all_points = np.asarray(pcd.points)
    for point in all_points:
        distance = distance_plane_to_point(point[0], point[1], point[2], a, b, c, d)

        if distance > 3 and ground_points[0][2] > point[2]:
            print(distance)

    # print(ground_points)
