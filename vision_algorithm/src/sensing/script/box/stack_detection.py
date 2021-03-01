import pcl
import cv2
from box_io import getImageArrayFast
from segment import plane_extract
from grasp_selection import grasp_selection
from pose import pose_estimation
from config import parser
from collision import grasper_collision_test, box_collision_test
from edge_detection import egde_detect
from conversion import get_edge_point_cloud, get_point_cloud_bb
import numpy as np
import struct
import time

params = parser.parse_args()
#  get camera intrinsic parameters
camera_paras = params.camera_intrinsics
intrinsics = np.identity(3, np.float32)
intrinsics[0, 0] = camera_paras[0]
intrinsics[1, 1] = camera_paras[1]
intrinsics[0, 2] = camera_paras[2]
intrinsics[1, 2] = camera_paras[3]

def getImageArray(pts, width, height):
    rgb_arr = pts[:, 3]
    getb = lambda t: struct.pack('>f', t)[-1]
    vfunc = np.vectorize(getb)
    b_arr = vfunc(rgb_arr)
    result = b_arr.reshape(height, width)
    return result.astype(np.uint8)


def detect(np_cloud, z_min, z_max, width, height):
    success, result = detect_with_view(np_cloud, width, height)
    if(success):
        _, _, pose, rec_wid, rec_len = result[0]
        print(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], rec_wid, rec_len)
        return True, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], rec_wid, rec_len

    print("No plane detected")
    return False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

def detect_with_view(pts, width, height):
    # 2. load image
    gray_img = getImageArray(pts, int(width), int(height))
    cv2.imwrite("result.jpg", gray_img)

    pts = pts[:, 0:3]
    print(pts.shape)

    # 3. segment point cloud
    grasp_box = pts
    cluster_planes, cloud_ds = plane_extract(pts, params)
    # print('time cost for segmentation: ', time.time() - time1)
    grasp_plane_ids = grasp_selection(cluster_planes, params)
    # visualize_scene(cluster_planes)

    if (len(cluster_planes) != 0):
        for i in range(len(grasp_plane_ids)):
            # 3.1 get the corresponding sub-image for this extracted plane
            idx = grasp_plane_ids[i]
            cluster_planes_i = cluster_planes[idx][:, 0:3]
            rmin, rmax, cmin, cmax = get_point_cloud_bb(cluster_planes_i, intrinsics)
            gray_masked = gray_img[rmin:rmax, cmin: cmax]

            # 3.2 get the edges in this sub-image
            # time1 = time.time()
            edges_masked = egde_detect(gray_masked, params.model)  # 2. detect edges from image
            edges = np.zeros_like(gray_img)
            edges[rmin:rmax, cmin: cmax] = edges_masked
            # print('time cost for detecting edges in the image: ', time.time() - time1)

            # 3.3 get the edge points
            edge_pts = get_edge_point_cloud(pts, intrinsics, edges)
            # print("edge_pts: ", edge_pts.shape)
            # np.savetxt("cluster_planes_i.txt", cluster_planes_i)

            # 3.4 filtering the neighboring points near the edge points
            # time1 = time.time()
            pc = pcl.PointCloud(cluster_planes_i)
            kd = pcl.KdTreeFLANN(pc)
            edge_pts = np.float32(edge_pts)
            pc_edges = pcl.PointCloud(edge_pts)
            indices, sqr_distances = kd.nearest_k_search_for_cloud(pc_edges, 50)
            indices = indices.flatten()
            cluster_planes_i_center = np.mean(cluster_planes_i, axis=0)
            cluster_planes_i[indices, :] = cluster_planes_i_center
            # print('time cost for filtering the neighboring points near the edge points: ', time.time() - time1)

            # 3.5 segment the cluster_planes_i
            sub_cluster_planes, _ = plane_extract(cluster_planes_i, params)

            # 3.6 grasp pose estimation
            if(len(sub_cluster_planes) > 1):
                grasp_box = sub_cluster_planes[0]
                # print("aaa")
                #visualize_scene_with_pose(sub_cluster_planes, 0, R, t)
                return True, [pose_estimation(sub_cluster_planes[0]), grasp_box, sub_cluster_planes, 0]
            else:
                grasp_box = cluster_planes_i
                # np.savetxt("cluster_planes_i.txt", cluster_planes_i)
                #visualize_scene_with_pose(cluster_planes, idx, R, t)
                # print("bbb")
                return True, [pose_estimation(cluster_planes_i), grasp_box, cluster_planes, idx]

    else:
        print("No any plane structures!")
        return False, []




