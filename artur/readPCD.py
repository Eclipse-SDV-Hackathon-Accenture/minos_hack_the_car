# coding:utf-8
import math

import open3d as o3d
import numpy as np
import copy
import matplotlib.pyplot as plt


def get_distance_from_point_to_line(pnt, line_point1, line_point2):
    # same points, return distance between points
    if line_point1 == line_point2:
        point_array = np.array(pnt)
        point1_array = np.array(line_point1)
        return np.linalg.norm(point_array - point1_array)
    A = line_point2[1] - line_point1[1]
    B = line_point1[0] - line_point2[0]
    C = (line_point1[1] - line_point2[1]) * line_point1[0] + \
        (line_point2[0] - line_point1[0]) * line_point1[1]
    distance = np.abs(A * pnt[0] + B * pnt[1] + C) / (np.sqrt(A**2 + B**2))
    return distance


def min_dis_to_boundary(boundary, pnt):
    min_dis = 1000
    N = len(boundary)
    for ii in range(N):
        d = math.sqrt((boundary[ii][0] - pnt[0])**2 + (boundary[ii][1] - pnt[1])**2)
        min_dis = min(min_dis, d)
    return min_dis


def is_intersect(l1, l2):
    #   l1 [xa, ya, xb, yb]   l2 [xa, ya, xb, yb]
    v1 = (l1[0] - l2[0], l1[1] - l2[1])
    v2 = (l1[0] - l2[2], l1[1] - l2[3])
    v0 = (l1[0] - l1[2], l1[1] - l1[3])
    a = v0[0] * v1[1] - v0[1] * v1[0]
    b = v0[0] * v2[1] - v0[1] * v2[0]

    temp = l1
    l1 = l2
    l2 = temp
    v1 = (l1[0] - l2[0], l1[1] - l2[1])
    v2 = (l1[0] - l2[2], l1[1] - l2[3])
    v0 = (l1[0] - l1[2], l1[1] - l1[3])
    c = v0[0] * v1[1] - v0[1] * v1[0]
    d = v0[0] * v2[1] - v0[1] * v2[0]

    if a*b < 0 and c*d < 0:
        return 1
    else:
        return 0


def inside_boundary(boundary, pnt):
    cnt = 0
    N = len(boundary)
    cnt += is_intersect([boundary[0][0], boundary[0][1], boundary[N-1][0], boundary[N-1][1]],
                        [0, 0, pnt[0], pnt[1]])
    for ii in range(N-1):
        cnt += is_intersect([boundary[ii][0], boundary[ii][1], boundary[ii+1][0], boundary[ii+1][1]],
                            [0, 0, pnt[0], pnt[1]])
    if cnt % 2 == 0:
        return True
    else:
        return False


def pcd_ground_seg_open3d(scan):
    """ Open3D also supports segmententation of geometric primitives from point clouds using RANSAC.
    """
    pcd = copy.deepcopy(scan)
    ground_model, ground_indexes = scan.segment_plane(distance_threshold=0.08,
                                                      ransac_n=5,
                                                      num_iterations=1000)
    ground_indexes = np.array(ground_indexes)
    ground = pcd.select_by_index(ground_indexes)
    rest = pcd.select_by_index(ground_indexes, invert=True)
    # ground.paint_uniform_color(config['ground_color'])
    # rest.paint_uniform_color(config['rest_color'])
    return ground, rest

def distance_plane_to_point(x1, y1, z1, a: float, b: float, c: float, d: float) -> float:
    d = abs((a * x1 + b * y1 + c * z1 + d))
    e = (math.sqrt(a * a + b * b + c * c))
    try:
        return d / e
    except FloatingPointError:
        return 0


if __name__ == "__main__":
    print("hello")
    # for i in range(20):
    #     point = o3d.io.read_point_cloud("test" + str(i*10) + ".pcd")
    #     o3d.visualization.draw_geometries([point])

    pcd = o3d.io.read_point_cloud("../data/car/test.pcd")
    # set three axis for visualization
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    mesh.scale(5, center=mesh.get_center())

    # segment the point cloud using a bbox
    filtered_points = [[], []]
    all_points = np.asarray(pcd.points)
    all_colors = np.asarray(pcd.colors)

    forward_thresh = 40
    backward_thresh = 10
    length = 20
    for i in range(0, len(all_points)):
        point = all_points[i]
        if -forward_thresh <= point[0] <= backward_thresh and -length <= point[1] <= length and point[2] <= -0.4:
            filtered_points[0].append(point)
            # filtered_points[1].append(all_colors[i])
    filtered_model = o3d.geometry.PointCloud()
    filtered_model.points = o3d.utility.Vector3dVector(np.array(filtered_points[0]))
    # o3d.visualization.draw_geometries([filtered_model], width=800, height=600)

    # first round of ground segmentation, with more points
    ground, rest = pcd_ground_seg_open3d(filtered_model)

    # filter out the noise of point cloud
    num_points = 5  # minimum neighbors
    radius = 0.1
    ror_pcd, ind = rest.remove_radius_outlier(num_points, radius)

    # remove hidden points, keep the front points only
    diameter = np.linalg.norm(
        np.asarray(filtered_model.get_max_bound()) - np.asarray(filtered_model.get_min_bound()))
    print("Define parameters used for hidden_point_removal")
    camera = [0, 0, diameter]
    radius = diameter * 70
    _, pt_map = filtered_model.hidden_point_removal(camera, radius)
    front_pc = filtered_model.select_by_index(pt_map)
    # o3d.visualization.draw_geometries([front_pc], width=800, height=600)

    # second round of ground point segmentation, with fewer points after removing hidden points
    grd_model, grd_idx = front_pc.segment_plane(distance_threshold=0.08,
                                                ransac_n=5,
                                                num_iterations=1000)
    ground_indexes = np.array(grd_idx)
    ground = front_pc.select_by_index(grd_idx)

    # filter the noise of ground point cloud
    num_points = 10
    radius = 0.5
    ground_ror, ind = ground.remove_radius_outlier(num_points, radius)
    # obb = ground_ror.get_oriented_bounding_box()
    # obb.color = (0, 1, 0)  # 绿色
    # o3d.visualization.draw_geometries([mesh, ground_ror], width=800, height=600)

    # define the 2D shape of the ground point cloud
    ground_points = np.asarray(ground_ror.points)
    # cut the point clouds vertically into P slices
    P = 60
    laser_group_dir = []
    for i in range(P):
        laser_group_dir.append([])
    for pt in ground_points:
        ang = math.atan2(pt[0], pt[1]) * 180.0 / math.pi + 180
        laser_group_dir[int(ang/(360/P))].append([pt[0], pt[1], pt[2]])
    # form the boundary of the ground point cloud
    boundary_point = []
    for grp in laser_group_dir:
        max_dis = 0
        far_pt = np.array([0, 0, 0])
        for pt in grp:
            dis = math.sqrt(pt[0]**2 + pt[1]**2)
            if dis > max_dis:
                max_dis = dis
                far_pt = pt
        if math.sqrt(far_pt[0]**2 + far_pt[1]**2) >= 0.1:
            boundary_point.append([far_pt[0], far_pt[1], far_pt[2]])
    # down sample the points
    boundary_pc = o3d.geometry.PointCloud()
    boundary_pc.points = o3d.utility.Vector3dVector(np.array(boundary_point))
    voxel_size = 2
    boundary_pc_dsp = boundary_pc.voxel_down_sample(voxel_size)
    # o3d.visualization.draw_geometries([mesh, boundary_pc_dsp], width=800, height=600)

    # mesh_tx = copy.deepcopy(mesh).translate((1.3, 0, 0))
    # mesh_ty = copy.deepcopy(mesh).translate((0, 1.3, 0))
    # print(f'Center of mesh: {mesh.get_center()}')

    # define a smaller region of interest and find bumps inside
    roi_points = [[], []]
    all_points = np.asarray(ror_pcd.points)
    all_colors = np.asarray(ror_pcd.colors)

    # Filter the points
    forward_thresh = 15
    backward_thresh = 5
    length = 4
    for i in range(0, len(all_points)):
        point = all_points[i]
        if -forward_thresh <= point[0] <= backward_thresh and -length <= point[1] <= length and point[2] <= -0.5:
            roi_points[0].append(point)

    roi_pc = o3d.geometry.PointCloud()
    roi_pc.points = o3d.utility.Vector3dVector(np.array(roi_points[0]))

    # cluster the point clouds within ROI
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            roi_pc.cluster_dbscan(eps=0.2, min_points=8, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    roi_pc.colors = o3d.utility.Vector3dVector(colors[:, :3])

    # cluster_color = roi_pc.colors
    # get each sub-point clouds of the clustering result
    cluster_idx = []
    for i in range(max_label+1):
        cluster_idx.append([])
    for i in range(len(labels)):
        c = labels[i]
        if c == -1:
            continue
        cluster_idx[c].append(i)

    #TODO: detection of height, distance and smoothness of the object
    #TODO: categorize how severe the situation is
    #TODO: slow down, change, stop
    #TODO: check the intensity of a measured cloud point

    # mathematical plane
    a, b, c, d = grd_model

    # detect if the cluster is a bump
    bump_bbox = []
    for idx in cluster_idx:
        if len(idx) > 100:
            continue
        cluster = roi_pc.select_by_index(idx)
        center = cluster.get_center()

        # distance to boundary
        dis_to_bdy = min_dis_to_boundary(boundary_pc_dsp.points, center)
        print(dis_to_bdy)

        # distance to plane
        dis_to_plane = distance_plane_to_point(center[0], center[1], center[2], a, b, c, d)

        if inside_boundary(boundary_pc_dsp.points, center) and 0.25 <= dis_to_plane <= 1.3 and dis_to_bdy > 0.7:
            obb = cluster.get_oriented_bounding_box()
            obb.color = (0, 1, 0)  # 绿色
            bump_bbox.append(obb)
    o3d.visualization.draw_geometries([roi_pc, boundary_pc_dsp] + bump_bbox, width=800, height=600)


