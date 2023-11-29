import open3d as o3d
import numpy as np

if __name__ == "__main__":
    np.seterr('raise')

    point = o3d.io.read_point_cloud("../data/car/test.pcd")

    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    mesh.scale(5, center=mesh.get_center())

    o3d.visualization.draw_geometries([point, mesh])