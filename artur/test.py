import open3d as o3d
import numpy as np

def bounding_box_to_point_cloud(bounding_box, num_points=1000):
    # Extract the corners of the bounding box
    corners = np.array(bounding_box.get_box_points())

    # Sample points uniformly within the bounding box
    points = np.random.uniform(corners.min(axis=0), corners.max(axis=0), size=(num_points, 3))

    # Create a point cloud
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    return point_cloud

# Example usage:
# Create a bounding box
bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-1, -1, -1), max_bound=(1, 1, 1))

# Convert the bounding box to a point cloud
point_cloud = bounding_box_to_point_cloud(bbox, num_points=1000)

# Visualize the point cloud
o3d.visualization.draw_geometries([point_cloud])