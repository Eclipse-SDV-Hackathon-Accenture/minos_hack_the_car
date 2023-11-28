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
    print("hello")
    # for i in range(20):
    #     point = o3d.io.read_point_cloud("test" + str(i*10) + ".pcd")
    #     o3d.visualization.draw_geometries([point])
    point = o3d.io.read_point_cloud("test" + str(20 * 10) + ".pcd")
    # o3d.visualization.draw_geometries([point])
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    mesh.scale(5, center=mesh.get_center())
    # mesh_tx = copy.deepcopy(mesh).translate((1.3, 0, 0))
    # mesh_ty = copy.deepcopy(mesh).translate((0, 1.3, 0))
    print(f'Center of mesh: {mesh.get_center()}')
    # print(f'Center of mesh tx: {mesh_tx.get_center()}')
    # print(f'Center of mesh ty: {mesh_ty.get_center()}')
    o3d.visualization.draw_geometries([mesh, point])
    # ground, rest = pcd_ground_seg_open3d(point)
    # o3d.visualization.draw_geometries([ground])
