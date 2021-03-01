import pcl
from visualize import *
from segment import plane_extract
from grasp_selection import grasp_selection
from pose import pose_estimation
from config import parser
from collision import grasper_collision_test, box_collision_test
# import numpy as np 

import time

params = parser.parse_args()
if __name__ == '__main__':
    ply_path = '/home/jiang/saisun_ws/data/ply_0.ply'
    # params.filter_z_max = 1.2
    # params.filter_z_min = 1.5

    cloud = pcl.load(ply_path)
    np_cloud = cloud.to_array()

    print(np_cloud.shape)
    # out_arr = np.asarray(cloud) 

    start_time = time.time()
    # cluster_planes, cloud_ds = plane_extract(out_arr, params)
    cluster_planes, cloud_ds = plane_extract(np_cloud, params)
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
            print(pose)
            # box collision detection
            box_range, box_range_lines, points_inbox, box_is_collision, overlap_points = box_collision_test(cloud_ds, R, t, rec_wid, rec_len, params)
            if(box_is_collision != 1):
                # grasper collision detection
                grasper_box, box_lines, points_inbox, box_graspable, overlap_points = grasper_collision_test(cloud_ds, R, t, params)
                if (box_graspable == 1):
                    grasp_id = cluster_id
                    box_is_collision = 0
                    break
            else:
                continue
            #visualize_scene_with_pose(cluster_planes, cluster_id, R, t)

    end_time2 = time.time()
    print('time cost for pose estimation:', end_time2 - end_time1)

    scene_pts = o3d.io.read_point_cloud(ply_path)
    visualize_scene_with_pose(cluster_planes, grasp_id, R, t)

    # # scene_pts = o3d.geometry.PointCloud()
    # # scene_pts.points = o3d.utility.Vector3dVector(cloud_ds)
    # if(box_graspable == 0):
    #     overlap = o3d.geometry.PointCloud()
    #     overlap.points = o3d.utility.Vector3dVector(overlap_points)
    #     visualize_pose_in_raw_pts_one_box(scene_pts, cluster_planes[grasp_id][:, 0:3], R, t, overlap, grasper_box, box_lines)
    # else:
    #     visualize_pose_in_raw_pts_two_box(scene_pts, cluster_planes[grasp_id][:, 0:3], R, t, grasper_box, box_lines, box_range, box_range_lines)

    print('poses:', pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], rec_wid, rec_len)
