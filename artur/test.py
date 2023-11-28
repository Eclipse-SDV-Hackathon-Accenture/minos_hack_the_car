import open3d as o3d
import numpy as np

# Load point cloud data
pcd = o3d.io.read_point_cloud("../data/car/test.pcd")

# Filter ground points using RANSAC
plane_model, inliers = pcd.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=1000)
ground_points = pcd.select_by_index(inliers)

# Identify holes in the road
clustering = np.asarray(ground_points.cluster_dbscan(eps=0.5, min_points=10, print_progress=False))
labels = clustering  # No need to separate the labels and clusters

ground = np.asarray(ground_points.points)
holes = []
for i in range(0, len(ground)):
    if labels[i] == -1:
        holes.append(ground[i])

hole_model = o3d.geometry.PointCloud()
hole_model.points = o3d.utility.Vector3dVector(holes)

# Visualize the results
o3d.visualization.draw_geometries([hole_model, ground_points])

# Print hole indices
print("Hole indices:", holes)
