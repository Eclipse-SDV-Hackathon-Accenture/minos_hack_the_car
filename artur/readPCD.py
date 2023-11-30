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
    distance = np.abs(A * pnt[0] + B * pnt[1] + C) / (np.sqrt(A ** 2 + B ** 2))
    return distance


def min_dis_to_boundary(boundary, pnt):
    min_dis = 1000
    N = len(boundary)
    for ii in range(N):
        d = math.sqrt((boundary[ii][0] - pnt[0]) ** 2 + (boundary[ii][1] - pnt[1]) ** 2)
        min_dis = min(min_dis, d)
    return min_dis


# check if two line intersect
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

    if a * b < 0 and c * d < 0:
        return 1
    else:
        return 0


# check if the point is inside a polygon
def inside_boundary(boundary, pnt):
    cnt = 0
    N = len(boundary)
    cnt += is_intersect([boundary[0][0], boundary[0][1], boundary[N - 1][0], boundary[N - 1][1]],
                        [0, 0, pnt[0], pnt[1]])
    for ii in range(N - 1):
        cnt += is_intersect([boundary[ii][0], boundary[ii][1], boundary[ii + 1][0], boundary[ii + 1][1]],
                            [0, 0, pnt[0], pnt[1]])
    if cnt % 2 == 0:
        return True
    else:
        return False


# ground points segmentation
def pcd_ground_seg_open3d(scan):
    pcd = copy.deepcopy(scan)
    # ground model consists of 4 numbers, a, b, c, d
    ground_model, ground_indexes = scan.segment_plane(distance_threshold=0.08,
                                                      ransac_n=5,
                                                      num_iterations=1000)
    ground_indexes = np.array(ground_indexes)
    ground = pcd.select_by_index(ground_indexes)
    rest = pcd.select_by_index(ground_indexes, invert=True)
    return ground, rest


def distance_plane_to_point(x1, y1, z1, a: float, b: float, c: float, d: float) -> float:
    d = abs((a * x1 + b * y1 + c * z1 + d))
    e = (math.sqrt(a * a + b * b + c * c))
    try:
        return d / e
    except FloatingPointError:
        return 0


def get_cluster_height(cluster):
    points = np.array(cluster.points)
    l_z, h_z = 0, 0

    for i in range(1, points.shape[0] - 1):
        if points[i][2] < points[l_z][2]:
            l_z = i
        if points[i][2] > points[h_z][2]:
            h_z = i
    return abs(points[h_z][2] - points[l_z][2])


def get_cluster_width(cluster):
    obb = cluster.get_oriented_bounding_box()
    points = np.array(obb.get_box_points())
    width = 0
    length = points.shape[0]

    for i in range(0, length - 1):
        x1, y1, z1 = points[i]
        for i in range(0, length - 1):
            x2, y2, z2 = points[i]

            dis = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
            if dis > width:
                width = dis
    return width


def cluster_highest_lowest_point(cluster):
    points = np.array(cluster.points)
    l_z, h_z = 0, 0

    for i in range(1, points.shape[0] - 1):
        if points[i][2] < points[l_z][2]:
            l_z = i
        if points[i][2] > points[h_z][2]:
            h_z = i
    return points[h_z], points[l_z]


def categorize_cluster(cluster):
    # slow down = 0; change = 1; stop = 2;
    warnings = ["slow down", "change", "stop"]

    categories = [[0, 0, 0], [0, 1, 2], [1, 1, 2]]

    w_opt = 0
    h_opt = 0

    # slimness 0.5m 1m 2m
    width = get_cluster_width(cluster)

    if 2 <= width:
        w_opt = 2
    elif 1 <= width:
        w_opt = 1
    elif 0.15 <= width:
        w_opt = 0

    # height
    height = get_cluster_height(cluster)

    if height < 0.25:
        h_opt = 0
    elif height < 0.35:
        h_opt = 1
    elif 0.35 <= height:
        h_opt = 2

    print(warnings[categories[h_opt][w_opt]])
    return warnings[categories[h_opt][w_opt]], categories[h_opt][w_opt]


def calculate_distance_to_obstacle(point):
    x, y, z = -1.2, 0, -0.5

    distance = math.sqrt((x - point[0]) ** 2 + (y - point[1]) ** 2 + (z - point[2]) ** 2)
    return distance


def find_obstacles(pcd, flag=0):

    # set three axis for visualization
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    mesh.scale(5, center=mesh.get_center())
    if flag:
        o3d.visualization.draw_geometries([mesh, pcd], width=800, height=600)

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
    if flag:
        o3d.visualization.draw_geometries([mesh, filtered_model], width=800, height=600)

    # first round of ground segmentation, with more points
    ground, rest = pcd_ground_seg_open3d(filtered_model)

    # filter out the noise of point cloud
    num_points = 5  # minimum neighbors
    radius = 0.1
    ror_pcd, ind = rest.remove_radius_outlier(num_points, radius)

    # remove hidden points, keep the front points only
    diameter = np.linalg.norm(
        np.asarray(filtered_model.get_max_bound()) - np.asarray(filtered_model.get_min_bound()))
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
    o3d.visualization.draw_geometries([mesh, ground_ror], width=800, height=600)

    # define the 2D shape of the ground point cloud
    ground_points = np.asarray(ground_ror.points)
    # cut the point clouds vertically into P slices
    P = 60
    laser_group_dir = []
    for i in range(P):
        laser_group_dir.append([])
    for pt in ground_points:
        ang = math.atan2(pt[0], pt[1]) * 180.0 / math.pi + 180
        laser_group_dir[int(ang / (360 / P))].append([pt[0], pt[1], pt[2]])
    # form the boundary of the ground point cloud
    boundary_point = []
    for grp in laser_group_dir:
        max_dis = 0
        far_pt = np.array([0, 0, 0])
        for pt in grp:
            dis = math.sqrt(pt[0] ** 2 + pt[1] ** 2)
            if dis > max_dis:
                max_dis = dis
                far_pt = pt
        if math.sqrt(far_pt[0] ** 2 + far_pt[1] ** 2) >= 0.1:
            boundary_point.append([far_pt[0], far_pt[1], far_pt[2]])
    # down sample the points
    boundary_pc = o3d.geometry.PointCloud()
    boundary_pc.points = o3d.utility.Vector3dVector(np.array(boundary_point))
    # voxel_size = 2
    # boundary_pc_dsp = boundary_pc.voxel_down_sample(voxel_size)
    # o3d.visualization.draw_geometries([mesh, boundary_pc_dsp], width=800, height=600)
    boundary_pc.paint_uniform_color([0, 0.3, 0])  # 点云颜色
    lines = []
    for jj in range(len(boundary_pc.points)):
        lines.append([jj, jj+1])
    lines[-1][1] = 0
    color = [[0, 1, 0] for i in range(len(lines))]
    lines_pcd = o3d.geometry.LineSet()
    lines_pcd.lines = o3d.utility.Vector2iVector(lines)
    lines_pcd.colors = o3d.utility.Vector3dVector(color)  # 线条颜色
    lines_pcd.points = o3d.utility.Vector3dVector(boundary_pc.points)

    # define a smaller region of interest and find bumps inside
    roi_points = [[], []]
    all_points = np.asarray(ror_pcd.points)
    all_colors = np.asarray(ror_pcd.colors)

    # Filter the points
    forward_thresh = 15
    backward_thresh = 2
    length = 3
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

    # get each sub-point clouds of the clustering result
    cluster_idx = []
    for i in range(max_label + 1):
        cluster_idx.append([])
    for i in range(len(labels)):
        c = labels[i]
        if c == -1:
            continue
        cluster_idx[c].append(i)

    # TODO: detection of height, distance and smoothness of the object
    # TODO: categorize how severe the situation is
    # TODO: slow down, change, stop
    # TODO: check the intensity of a measured cloud point
    # TODO: Implement function to change the different frames

    # mathematical plane
    a, b, c, d = grd_model

    # detect if the cluster is a bump
    bump_bbox = []
    warning_str = []
    category_idx = []

    for idx in cluster_idx:
        # if len(idx) > 250:
        #     continue
        cluster = roi_pc.select_by_index(idx)
        center = cluster.get_center()

        # distance to boundary
        dis_to_bdy = min_dis_to_boundary(boundary_pc.points, center)

        # get cluster lowest and highest point
        highest, lowest = cluster_highest_lowest_point(cluster)

        # distance to plane
        dis_to_plane = distance_plane_to_point(center[0], center[1], center[2], a, b, c, d)
        height = distance_plane_to_point(highest[0], highest[1], highest[2], a, b, c, d)

        # get cluster height
        cluster_height = get_cluster_height(cluster)

        # if inside_boundary(boundary_pc.points, center) and -0.05 <= dis_to_plane <= 0.5 and
        # highest[2] <= 1.3 and dis_to_bdy > 1 and 0.15 < cluster_height:
        if inside_boundary(boundary_pc.points, center) and (abs(dis_to_plane) >= 0.1 or abs(height) > 0.2) and \
                dis_to_bdy > 0.8:
            warn, cat = categorize_cluster(cluster)
            warning_str.append(warn)
            category_idx.append(cat)

            distance_obstacle = calculate_distance_to_obstacle(center)
            # print("Distance: ", distance_obstacle)

            obb = cluster.get_oriented_bounding_box()
            obb.color = (0, 1, 0)  # green
            bump_bbox.append(obb)
    if flag:
        o3d.visualization.draw_geometries([mesh, roi_pc, lines_pcd] + bump_bbox, width=800, height=600)

    return roi_pc, warning_str, category_idx


"""
# main function for testing
def main():
    pcd_path = "../1701274679.755217000.pcd"
    pcd_path = "../1701274688.155573000.pcd"
    pcd_path = "../1701274707.255352000.pcd"
    pcd = o3d.io.read_point_cloud(pcd_path)
    find_obstacles(pcd, 0)


if __name__ == "__main__":
    main()
"""
