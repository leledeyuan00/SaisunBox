import pcl
from segment import plane_extract
from grasp_selection import grasp_selection
from pose import pose_estimation
from config import parser
import numpy as np
from collision import grasper_collision_test, box_collision_test
import struct
import time
import cv2
import pcl2_img

params = parser.parse_args()

def detect(np_cloud, z_min, z_max):
    params.filter_z_max = z_min
    params.filter_z_min = z_max
    print(np_cloud.shape)
    
    image_arr = getImageArray(np_cloud, int(width), int(height))
    cv2.imwrite("result.jpg", image_arr)

    #delete the intensity
    np_cloud = np.delete(np_cloud, 3, 1)
    print(np_cloud.shape)

    #delete nan rows
    mask = np.all(np.isnan(np_cloud), axis=1)
    processed_np_cloud = np_cloud[~mask]
    print(processed_np_cloud.shape)

    return detectCloud(processed_np_cloud)
    # cloud = pcl.PointCloud(processed_cloud)
    # return detectCloud(cloud)


def detectCloud(np_cloud):
    #convert to cloud
    
    start_time = time.time()
    cluster_planes, cloud_ds = plane_extract(np_cloud, params)
    if(len(cluster_planes) == 0):
        print("No plane detected!")
        return False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 
    # visualize_scene(cluster_planes)

    grasp_ids = grasp_selection(cluster_planes, params)
    # grasp_ids = list(range(len(cluster_planes)))
    # visualize_scene_with_optimal_plane(cluster_planes, grasp_id)
    end_time1 = time.time()
    print('time cost for plane segmentation:', end_time1 - start_time)

    grasper_box = []
    box_lines = []
    box_range = []
    box_range_lines = []
    box_is_collision = 0
    box_graspable = 1
    grasp_id = 0
    if (len(grasp_ids) == 1):
        grasp_id = 0
        R, t, pose, rec_wid, rec_len = pose_estimation(cluster_planes[0])
    else:
        for idx in range(len(grasp_ids)):
            cluster_id = grasp_ids[idx]
            R, t, pose, rec_wid, rec_len = pose_estimation(cluster_planes[cluster_id])
            # box collision detection
            box_range, box_range_lines, points_inbox, box_is_collision, overlap_points = box_collision_test(cloud_ds, R, t, rec_len, rec_wid, params)
            if(box_is_collision != 1):
                # grasper collision detection
                grasper_box, box_lines, points_inbox, box_graspable, overlap_points = grasper_collision_test(cloud_ds, R, t, params)
                if (box_graspable == 1):
                    grasp_id = cluster_id
                    box_is_collision = 0
                    break
            else:
                continue
            # visualize_scene_with_pose(cluster_planes, cluster_id, R, t)

    end_time2 = time.time()
    print('time cost for pose estimation:', end_time2 - end_time1)

    #scene_pts = o3d.io.read_point_cloud(ply_path)
    # visualize_scene_with_pose(cluster_planes, grasp_id, R, t)

    # scene_pts = o3d.geometry.PointCloud()
    # scene_pts.points = o3d.utility.Vector3dVector(cloud_ds)
    # if(box_graspable == 0):
    #     overlap = o3d.geometry.PointCloud()
    #     overlap.points = o3d.utility.Vector3dVector(overlap_points)
    #     visualize_pose_in_raw_pts_one_box(scene_pts, cluster_planes[grasp_id][:, 0:3], R, t, overlap, grasper_box, box_lines)
    # else:
    #     visualize_pose_in_raw_pts_two_box(scene_pts, cluster_planes[grasp_id][:, 0:3], R, t, grasper_box, box_lines, box_range, box_range_lines)

    # scene_pts = o3d.io.read_point_cloud(ply_path)
    # visualize_pose_in_raw_pts(scene_pts, cluster_planes[grasp_id][:, 0:3], R, t)
    print(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], rec_wid, rec_len)
    return True, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], rec_wid, rec_len

