import pcl
import cv2
from box_io import load_ply, getImageArrayFast
from visualize import *
from segment import plane_extract
from grasp_selection import grasp_selection
from pose import pose_estimation
from config import parser
from collision import grasper_collision_test, box_collision_test
from edge_detection import egde_detect
from conversion import get_edge_point_cloud, get_point_cloud_bb

import time

params = parser.parse_args()
#  get camera intrinsic parameters
camera_paras = params.camera_intrinsics
intrinsics = np.identity(3, np.float32)
intrinsics[0, 0] = camera_paras[0]
intrinsics[1, 1] = camera_paras[1]
intrinsics[0, 2] = camera_paras[2]
intrinsics[1, 2] = camera_paras[3]

# main program
if __name__ == '__main__':
    # 1. load pcd
    ply_path = '/home/conicacui/dev_ws/src/hkclr_smart_grasping_platform/grasping_platform/data/ply/model5.pcd'
    model_path = '/home/conicacui/dev_ws/src/hkclr_smart_grasping_platform/grasping_platform/script/box/model.yml'
    cloud = load_ply(ply_path)
    pts = cloud.to_array()

    # 2. load image
    time1 = time.time()
    gray_img = getImageArrayFast(pts, cloud.width, cloud.height) # point cloud to image
    print('time cost for converting image: ', time.time() - time1)

    
    pts = pts[:, 0:3]

    # 3. segment point cloud
    grasp_box = pts
    cluster_planes, cloud_ds = plane_extract(pts, params)
    # print('time cost for segmentation: ', time.time() - time1)
    grasp_plane_ids = grasp_selection(cluster_planes, params)
    # visualize_scene(cluster_planes)

    if (len(grasp_plane_ids) != 0):
        for i in range(len(grasp_plane_ids)):
            # 3.1 get the corresponding sub-image for this extracted plane
            idx = grasp_plane_ids[i]
            cluster_planes_i = cluster_planes[idx][:, 0:3]
            rmin, rmax, cmin, cmax = get_point_cloud_bb(cluster_planes_i, intrinsics)
            gray_masked = gray_img[rmin:rmax, cmin: cmax]

            # 3.2 get the edges in this sub-image
            # time1 = time.time()
            edges_masked = egde_detect(gray_masked, model_path)  # 2. detect edges from image
            edges = np.zeros_like(gray_img)
            edges[rmin:rmax, cmin: cmax] = edges_masked
            # print('time cost for detecting edges in the image: ', time.time() - time1)

            # 3.3 get the edge points
            edge_pts = get_edge_point_cloud(pts, intrinsics, edges)
            # np.savetxt("edge_points.txt", edge_pts)

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
                R, t, pose, rec_len, rec_wid = pose_estimation(sub_cluster_planes[0])
                visualize_scene_with_pose(sub_cluster_planes, 0, R, t)
                break
            else:
                grasp_box = cluster_planes_i
                R, t, pose, rec_len, rec_wid = pose_estimation(cluster_planes_i)
                # np.savetxt("cluster_planes_i.txt", cluster_planes_i)
                visualize_scene_with_pose(cluster_planes, idx, R, t)
                break

        # 3.7 visulization in the scene
        print('total time: ', time.time() - time1)
        scene = o3d.geometry.PointCloud()
        scene.points = o3d.utility.Vector3dVector(pts)
        visualize_pose_in_raw_pts(scene, grasp_box[:, 0:3], R, t)

    else:
        print("No any plane structures!")
