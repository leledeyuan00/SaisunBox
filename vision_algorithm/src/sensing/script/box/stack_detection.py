import pcl
import cv2
from segment import plane_extract, is_perp_plane
from grasp_selection import sort_with_height
from pose import pose_estimation
from config import parser
from collision import grasper_collision_test, box_collision_test
from edge_detection import egde_detect
from conversion import get_edge_point_cloud, get_point_cloud_bb
# from visualize import visualize_scene
import numpy as np
import struct
import time
import pcl2_img

params = parser.parse_args()
#  get camera intrinsic parameters
camera_paras = params.camera_intrinsics
intrinsics = np.identity(3, np.float32)
intrinsics[0, 0] = camera_paras[0]
intrinsics[1, 1] = camera_paras[1]
intrinsics[0, 2] = camera_paras[2]
intrinsics[1, 2] = camera_paras[3]

def detect(np_cloud, gray_img):
    gray_img = pcl2_img.rgb2gray(gray_img)
    success, result = detect_with_view(np_cloud, gray_img)
    if(success):
        _, _, pose, rec_wid, rec_len = result[0]
        print(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], rec_wid, rec_len)
        return True, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], rec_wid, rec_len

    print("No plane detected")
    return False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

def detect_with_view(pts, gray_img):

    pts = pts[:, 0:3]
    print(pts.shape)

    # 3. segment point cloud
    grasp_box = pts
    cluster_planes, cloud_ds = plane_extract(pts, params)
    # print('time cost for segmentation: ', time.time() - time1)
    grasp_plane_ids = sort_with_height(cluster_planes, params)
    # visualize_scene(cluster_planes)
    print("grasp plan len is:" + str(len(grasp_plane_ids)))
    print("cluster plan len is:" + str(len(cluster_planes)))
    if (len(cluster_planes) != 0):
        for i in range(len(grasp_plane_ids)):
            # 3.1 get the corresponding sub-image for this extracted plane
            idx = grasp_plane_ids[i]
            cluster_planes_i = cluster_planes[idx][:, 0:3]
            if(is_perp_plane(cluster_planes_i, params.filter_ground_plane_para)):
                continue
                # print("perpendicular")
            else:
                # rmin, rmax, cmin, cmax = get_point_cloud_bb(cluster_planes_i, intrinsics)
                rmin, rmax, cmin, cmax = get_point_cloud_bb(cluster_planes_i, pts, gray_img.shape)
                gray_masked = gray_img[rmin:rmax, cmin:cmax]

                # 3.2 get the edges in this sub-image
                # time1 = time.time()
                edges_masked = egde_detect(gray_masked, params.model)  # 2. detect edges from image
                edges = np.zeros_like(gray_img)
                edges[rmin:rmax, cmin: cmax] = edges_masked
                # print('time cost for detecting edges in the image: ', time.time() - time1)

                # 3.3 get the edge points
                edge_pts = get_edge_point_cloud(pts, edges)
                # print("edge_pts: ", edge_pts.shape)
                np.savetxt("edge_pts.txt", edge_pts)

                # 3.4 filtering the neighboring points near the edge points
                # time1 = time.time()
                pc = pcl.PointCloud(cluster_planes_i)
                kd = pcl.KdTreeFLANN(pc)
                edge_pts = np.float32(edge_pts)
                pc_edges = pcl.PointCloud(edge_pts)
                # indices, sqr_distances = kd.nearest_k_search_for_cloud(pc_edges, 25)
                indices, sqr_distances = kd.radius_search_for_cloud(pc_edges, params.Radius_KD, 25)
                # print("indice: ", indices.shape)
                # indices, sqr_distances = kd.radius_search_for_cloud(pc_edges, 0.009)
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




