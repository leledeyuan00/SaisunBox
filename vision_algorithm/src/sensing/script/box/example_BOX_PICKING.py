import pcl
from box_io import load_ply
from visualize import *
from segment import plane_extract
from grasp_selection import grasp_selection
from pose import pose_estimation
from config import parser
from collision import grasper_collision_test, box_collision_test
from edge_detection import egde_detect
from depth_convert import get_edge_point_cloud
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
    ply_path = '../../../../data/model5.pcd'
    pts = load_ply(ply_path)
    start_time = time.time()
    if(params.UNSTACK):
        img_path = './data/20200220/leftView5.bmp'
        time1 = time.time()
        edges = egde_detect(img_path) # 2. detect edges from image
        print('time cost for detecting edges in the image: ', time.time() - time1)

        # 2. get the edge point coordinates-

        time1 = time.time()
        edge_pts = get_edge_point_cloud(pts, intrinsics, edges)
        # np.savetxt("edge_points.txt", edge_pts)
        print('time cost for getting edge pts: ', time.time() - time1)
        # 3. filtering the neighboring points near the edge points
        time1 = time.time()
        pc = pcl.PointCloud(pts)
        kd = pcl.KdTreeFLANN(pc)
        edge_pts = np.float32(edge_pts)
        pc_edges = pcl.PointCloud(edge_pts)
        indices, sqr_distances = kd.nearest_k_search_for_cloud(pc_edges, 100)
        indices = indices.flatten()
        pts[indices, :] = [0, 0, 0]
        print('time cost for filtering the neighboring points near the edge points: ', time.time() - time1)

    # 4. plane segmentation
    time1 = time.time()
    cluster_planes, cloud_ds = plane_extract(pts, params)
    print('time cost for plane segmentation:', time.time() - time1)
    # visualize_scene(cluster_planes)

    # 5. select a best grasp plane
    grasp_ids = grasp_selection(cluster_planes, params)
    end_time1 = time.time()

    grasper_box = []
    box_lines = []
    box_range = []
    box_range_lines = []
    box_is_collision = 0
    box_graspable = 1
    grasp_id = 0
    if (len(grasp_ids) == 1):
        grasp_id = 0
        R, t, pose, rec_len, rec_wid = pose_estimation(cluster_planes[0])
    else:
        for idx in range(len(grasp_ids)):
            cluster_id = grasp_ids[idx]
            R, t, pose, rec_len, rec_wid = pose_estimation(cluster_planes[cluster_id])
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
    print('Total time cost:', end_time2 - start_time)

    scene_pts = o3d.io.read_point_cloud(ply_path)
    visualize_scene_with_pose(cluster_planes, grasp_id, R, t)

    if(box_graspable == 0):
        overlap = o3d.geometry.PointCloud()
        overlap.points = o3d.utility.Vector3dVector(overlap_points)
        visualize_pose_in_raw_pts_one_box(scene_pts, cluster_planes[grasp_id][:, 0:3], R, t, overlap, grasper_box, box_lines)
    else:
        visualize_pose_in_raw_pts_two_box(scene_pts, cluster_planes[grasp_id][:, 0:3], R, t, grasper_box, box_lines, box_range, box_range_lines)
