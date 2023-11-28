import open3d as o3d
import numpy as np

if __name__ == "__main__":
    np.seterr('raise')

    point = o3d.io.read_point_cloud("../data/car/test.pcd")
    o3d.visualization.draw_geometries([point])