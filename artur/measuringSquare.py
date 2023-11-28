import math

import open3d as o3d
import numpy as np
import copy


def add_point(model, point, color):
    model.points = o3d.utility.Vector3dVector(np.vstack([np.asarray(model.points), point]))
    model.colors = o3d.utility.Vector3dVector(np.vstack([np.asarray(model.colors), color]))


if __name__ == "__main__":
    np.seterr('raise')

    pcd = o3d.io.read_point_cloud("../data/laserscanner2/04083.pcd")

    # center, x, y, z
    red_point = np.array([[0, 0, 0], [5, 0, 0], [0, 5, 0], [0, 0, 5]])
    red_color = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 1.0, 0.0]])

    # Add the red point to the existing point cloud
    add_point(pcd, red_point, red_color)

    all_points = np.asarray(pcd.points)
    all_colors = np.asarray(pcd.colors)

    # o3d.visualization.draw_geometries([pcd])

    filtered_points = [[], []]

    # Filter the points
    width = 50
    length = 200
    for i in range(0, len(all_points)):
        point = all_points[i]
        if -(width/2) <= point[0] <= width/2 and -length <= point[1] and point[2] <= 0:
            filtered_points[0].append(point)
            filtered_points[1].append(all_colors[i])

    filtered_model = o3d.geometry.PointCloud()
    filtered_model.points = o3d.utility.Vector3dVector(np.array(filtered_points[0]))
    filtered_model.colors = o3d.utility.Vector3dVector(np.array(filtered_points[1]))

    o3d.visualization.draw_geometries([filtered_model])
