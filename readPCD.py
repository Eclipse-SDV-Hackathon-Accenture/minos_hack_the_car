# coding:utf-8

import open3d as o3d
import numpy as np
import copy


def pcd_ground_seg_open3d(scan):
    """ Open3D also supports segmententation of geometric primitives from point clouds using RANSAC.
    """
    pcd = copy.deepcopy(scan)
    ground_model, ground_indexes = scan.segment_plane(distance_threshold=0.3,
                                                      ransac_n=3,
                                                      num_iterations=100)
    ground_indexes = np.array(ground_indexes)
    ground = pcd.select_by_index(ground_indexes)
    rest = pcd.select_by_index(ground_indexes, invert=True)
    # ground.paint_uniform_color(config['ground_color'])
    # rest.paint_uniform_color(config['rest_color'])
    return ground, rest


if __name__ == "__main__":
    point = o3d.io.read_point_cloud("../data/laserscanner2/04082.pcd")
    # o3d.visualization.draw_geometries([point])
    ground, rest = pcd_ground_seg_open3d(point)
    o3d.visualization.draw_geometries([ground])
